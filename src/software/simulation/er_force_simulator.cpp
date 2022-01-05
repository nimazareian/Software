#include "software/simulation/er_force_simulator.h"

#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>

#include <QtCore/QFile>
#include <QtCore/QString>
#include <iostream>

#include "extlibs/er_force_sim/src/protobuf/robot.h"
#include "proto/message_translation/primitive_google_to_nanopb_converter.h"
#include "proto/message_translation/ssl_detection.h"
#include "proto/message_translation/ssl_geometry.h"
#include "proto/message_translation/ssl_simulation_robot_control.h"
#include "proto/message_translation/ssl_wrapper.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "software/world/robot_state.h"

ErForceSimulator::ErForceSimulator(
    const Field& field, const RobotConstants_t& robot_constants,
    const WheelConstants& wheel_constants,
    std::shared_ptr<const SimulatorConfig> simulator_config)
    : yellow_team_world_msg(std::make_unique<TbotsProto::World>()),
      blue_team_world_msg(std::make_unique<TbotsProto::World>()),
      frame_number(0),
      robot_constants(robot_constants),
      wheel_constants(wheel_constants)
{
    QString full_filename = CONFIG_DIRECTORY + CONFIG_FILE + ".txt";
    QFile file(full_filename);
    if (!file.open(QFile::ReadOnly))
    {
        LOG(FATAL) << "Could not open configuration file " << full_filename.toStdString()
                   << std::endl;
    }
    QString str = file.readAll();
    file.close();
    std::string s = qPrintable(str);

    google::protobuf::TextFormat::Parser parser;
    parser.ParseFromString(s, &er_force_sim_setup);

    er_force_sim = std::make_unique<camun::simulator::Simulator>(er_force_sim_setup);

    auto simulator_setup_command = std::make_unique<amun::Command>();
    simulator_setup_command->mutable_simulator()->set_enable(true);
    // start with default robots, take ER-Force specs.
    robot::Specs ERForce;
    robotSetDefault(&ERForce);

    Team friendly_team = Team();
    Team enemy_team    = Team();
    Ball ball          = Ball(Point(), Vector(), Timestamp::fromSeconds(0));

    World world = World(field, ball, friendly_team, enemy_team);

    // TODO (#2283): remove this initialization when addYellowRobots and addBlueRobots are
    // implemented
    auto* team_blue   = simulator_setup_command->mutable_set_team_blue();
    auto* team_yellow = simulator_setup_command->mutable_set_team_yellow();
    for (auto* team : {team_blue, team_yellow})
    {
        for (int i = 0; i < 11; ++i)
        {
            auto* robot = team->add_robot();
            robot->CopyFrom(ERForce);
            robot->set_id(i);
        }
    }
    er_force_sim->handleSimulatorSetupCommand(simulator_setup_command);
    er_force_sim->seedPRGN(17);

    // TODO (#2283): remove this initialization when addYellowRobots and addBlueRobots are
    // implemented
    for (unsigned int i = 0; i < 11; i++)
    {
        auto blue_simulator_robot = std::make_shared<ErForceSimulatorRobot>(
            RobotStateWithId{.id          = i,
                             .robot_state = RobotState(Point(), Vector(), Angle::zero(),
                                                       AngularVelocity::zero())},
            robot_constants, wheel_constants);
        auto yellow_simulator_robot = std::make_shared<ErForceSimulatorRobot>(
            RobotStateWithId{.id          = i,
                             .robot_state = RobotState(Point(), Vector(), Angle::zero(),
                                                       AngularVelocity::zero())},
            robot_constants, wheel_constants);

        blue_simulator_robots.emplace_back(blue_simulator_robot);
        yellow_simulator_robots.emplace_back(yellow_simulator_robot);
    }

    this->resetCurrentTime();
}

void ErForceSimulator::setBallState(const BallState& ball_state)
{
    auto simulator_setup_command = std::make_unique<amun::Command>();
    auto teleport_ball           = std::make_unique<sslsim::TeleportBall>();
    auto simulator_control       = std::make_unique<sslsim::SimulatorControl>();
    auto command_simulator       = std::make_unique<amun::CommandSimulator>();

    teleport_ball->set_x(
        static_cast<float>(ball_state.position().x() * MILLIMETERS_PER_METER));
    teleport_ball->set_y(
        static_cast<float>(ball_state.position().y() * MILLIMETERS_PER_METER));
    teleport_ball->set_vx(
        static_cast<float>(ball_state.velocity().x() * MILLIMETERS_PER_METER));
    teleport_ball->set_vy(
        static_cast<float>(ball_state.velocity().y() * MILLIMETERS_PER_METER));
    *(simulator_control->mutable_teleport_ball())   = *teleport_ball;
    *(command_simulator->mutable_ssl_control())     = *simulator_control;
    *(simulator_setup_command->mutable_simulator()) = *command_simulator;


    er_force_sim->handleSimulatorSetupCommand(simulator_setup_command);
}

void ErForceSimulator::addYellowRobots(const std::vector<RobotStateWithId>& robots)
{
    // TODO (#2283): add robots
}

void ErForceSimulator::addBlueRobots(const std::vector<RobotStateWithId>& robots)
{
    // TODO (#2283): add robots
}

void ErForceSimulator::setYellowRobotPrimitiveSet(
    const TbotsProto::PrimitiveSet& primitive_set_msg,
    std::unique_ptr<TbotsProto::World> world_msg)
{
    for (auto& [robot_id, primitive] : primitive_set_msg.robot_primitives())
    {
        setRobotPrimitive(robot_id, primitive, yellow_simulator_robots,
                          *yellow_team_world_msg);
    }
    // Use same world for both Yellow(friendly) and Blue(enemy) team, since we will need
    // all the info per robot. just flip teams
    yellow_team_world_msg = std::move(world_msg);  // why move if its a pointer?? it might allow it to not get deleted due to out of scope
}

void ErForceSimulator::setBlueRobotPrimitiveSet(
        const TbotsProto::PrimitiveSet& primitive_set_msg,
        std::unique_ptr<TbotsProto::World> world_msg)
{
    for (auto& [robot_id, primitive] : primitive_set_msg.robot_primitives())
    {
        setRobotPrimitive(robot_id, primitive, blue_simulator_robots,
                          *blue_team_world_msg);
    }
    blue_team_world_msg = std::move(world_msg);
}

void ErForceSimulator::setRobotPrimitive(
        RobotId id, const TbotsProto::Primitive& primitive_msg,
        std::vector<std::shared_ptr<ErForceSimulatorRobot>>& simulator_robots,
        const TbotsProto::World &world_msg)
{
    // Set to NEG_X because the vision msg in this simulator is normalized
    // correctly
    auto simulator_robots_iter =
        std::find_if(simulator_robots.begin(), simulator_robots.end(),
                     [id](const auto& robot) { return robot->getRobotId() == id; });

    if (simulator_robots_iter != simulator_robots.end())
    {
        auto simulator_robot = *simulator_robots_iter;

        const auto& friendly_robots = world_msg.friendly_team().team_robots();
        auto robot_iter =
                std::find_if(friendly_robots.begin(), friendly_robots.end(),
                             [id](const auto& robot) { return robot.id() == id; });

        if (robot_iter != friendly_robots.end())
        {
            simulator_robot->setRobotState(RobotState(robot_iter->current_state()));
            simulator_robot->startNewPrimitive(primitive_msg);
        }
        else
        {
            LOG(WARNING) << "Friendly robot with ID " << id << " not found" << std::endl;
        }
    }
    else
    {
        LOG(WARNING) << "Simulator robot with ID " << id << " not found" << std::endl;
    }
}

SSLSimulationProto::RobotControl ErForceSimulator::updateSimulatorRobots(
        const std::vector<std::shared_ptr<ErForceSimulatorRobot>>& simulator_robots,
        const TbotsProto::World& world_msg)
{
    SSLSimulationProto::RobotControl robot_control;

    for (auto& simulator_robot : simulator_robots)
    {
        const auto& friendly_robots = world_msg.friendly_team().team_robots();
        auto robot_iter =
                std::find_if(friendly_robots.begin(), friendly_robots.end(),
                             [simulator_robot](const auto& robot) { return robot.id() == simulator_robot->getRobotId(); });
        if (robot_iter != friendly_robots.end())
        {
            simulator_robot->setRobotState(
                RobotState(robot_iter->current_state()));
            // Set to NEG_X because the vision msg in this simulator is
            // normalized correctly
            simulator_robot->runCurrentPrimitive();
            auto command = *simulator_robot->getRobotCommand();
            *(robot_control.mutable_robot_commands()->Add()) = command;
        }
    }
    return robot_control;
}

void ErForceSimulator::stepSimulation(const Duration& time_step)
{
    current_time = current_time + time_step;

    SSLSimulationProto::RobotControl yellow_robot_control =
        updateSimulatorRobots(yellow_simulator_robots, *yellow_team_world_msg);

    SSLSimulationProto::RobotControl blue_robot_control =
        updateSimulatorRobots(blue_simulator_robots, *blue_team_world_msg);

    er_force_sim->acceptYellowRobotControlCommand(yellow_robot_control);
    er_force_sim->acceptBlueRobotControlCommand(blue_robot_control);
    er_force_sim->stepSimulation(time_step.toSeconds());

    frame_number++;
}

std::vector<SSLProto::SSL_WrapperPacket> ErForceSimulator::getSSLWrapperPackets() const
{
    return er_force_sim->getWrapperPackets();
}

Field ErForceSimulator::getField() const
{
    return Field::createSSLDivisionAField();
}

Timestamp ErForceSimulator::getTimestamp() const
{
    return current_time;
}

void ErForceSimulator::resetCurrentTime()
{
    current_time = Timestamp::fromSeconds(0);
}
