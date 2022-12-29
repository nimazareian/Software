#include "software/jetson_nano/primitive_executor.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/math/math_functions.h"
#include "proto/message_translation/tbots_protobuf.h"

PrimitiveExecutor::PrimitiveExecutor(const double time_step,
                                     const RobotConstants_t& robot_constants,
                                     const TeamColour friendly_team_colour)
    : current_primitive_(),
      robot_constants_(robot_constants),
      hrvo_simulator_(static_cast<float>(time_step), robot_constants,
                      friendly_team_colour),
      time_step_s_(time_step),
      curr_angular_velocity_(AngularVelocity::zero()),
      friendly_team_colour(friendly_team_colour),
      team_color(friendly_team_colour == TeamColour::YELLOW ? "y" : "b"),
      angular_speed_pid_(time_step,
                       3 * robot_constants.robot_max_ang_acceleration_rad_per_s_2 * time_step,
                       -3 * robot_constants.robot_max_ang_acceleration_rad_per_s_2 * time_step,
                         0.8,
                         0.09,
                         0.0),
     linear_speed_x_pid_(time_step,
                       3*robot_constants.robot_max_acceleration_m_per_s_2 * time_step,
                       -3*robot_constants.robot_max_ang_acceleration_rad_per_s_2 * time_step,
                       1,
                       0,
                       0),
      linear_speed_y_pid_(time_step,
                          3*robot_constants.robot_max_acceleration_m_per_s_2 * time_step,
                          -3*robot_constants.robot_max_ang_acceleration_rad_per_s_2 * time_step,
                          1,
                          0,
                          0)
{
}

void PrimitiveExecutor::updatePrimitiveSet(
    const unsigned int robot_id, const TbotsProto::PrimitiveSet& primitive_set_msg)
{
    hrvo_simulator_.updatePrimitiveSet(primitive_set_msg);
    auto primitive_set_msg_iter = primitive_set_msg.robot_primitives().find(robot_id);
    if (primitive_set_msg_iter != primitive_set_msg.robot_primitives().end())
    {
        current_primitive_ = primitive_set_msg_iter->second;
        return;
    }
}

void PrimitiveExecutor::clearCurrentPrimitive()
{
    current_primitive_.Clear();
}

void PrimitiveExecutor::updateWorld(const TbotsProto::World& world_msg)
{
    hrvo_simulator_.updateWorld(World(world_msg));
}

void PrimitiveExecutor::updateAngularVelocity(AngularVelocity angular_velocity)
{
    curr_angular_velocity_ = angular_velocity;
}

void PrimitiveExecutor::updateLocalVelocity(Vector local_velocity) {}

Vector PrimitiveExecutor::getTargetLinearVelocity(const unsigned int robot_id,
                                                  const Angle& curr_orientation)
{
    Vector target_global_velocity = hrvo_simulator_.getRobotVelocity(robot_id);
    return target_global_velocity.rotate(-curr_orientation);
}

Vector PrimitiveExecutor::getTargetLinearVelocity(
    const TbotsProto::MovePrimitive& move_primitive, const RobotState& robot_state)
{
    const Point final_position =
        createPoint(move_primitive.motion_control().path().points().at(1));

    const double x_diff = (robot_state.position() - final_position).x();
    const double y_diff = (robot_state.position() - final_position).y();

    const double x_inc = linear_speed_x_pid_.calculate(x_diff, 0.0);
    const double y_inc = linear_speed_y_pid_.calculate(y_diff, 0.0);

    return Vector(robot_state.velocity().x()+x_inc, robot_state.velocity().y()+y_inc).rotate(-robot_state.orientation());
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity(
    const TbotsProto::MovePrimitive& move_primitive, const Angle& curr_orientation)
{
    const Angle dest_orientation = createAngle(move_primitive.final_angle());
    const double signed_delta_orientation =
            (dest_orientation - curr_orientation).clamp().toRadians();

    // TODO: Should we be using feedback from the robot here: curr_angular_velocity_?
    const double inc = angular_speed_pid_.calculate(signed_delta_orientation, 0.0);
    AngularVelocity output = AngularVelocity::fromRadians(curr_angular_velocity_.toRadians() + inc);

    // Used to stop Jitter when at destination
    // Value determined experimentally
    if (output.abs().toRadians() < 0.15)
    {
        output = AngularVelocity::zero();
    }

    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt_actual", curr_angular_velocity_.toRadians()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt_output", output.toRadians()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt_pid_inc", inc});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_t_desired", dest_orientation.toRadians()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_t_signedDeltaToDest", signed_delta_orientation});

    curr_angular_velocity_ = output;

    // TODO: Nima remove support for turning
    return AngularVelocity::zero(); // output
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(
    const unsigned int robot_id,
    const RobotState& robot_state)
{
    robot_id_ = robot_id;
    hrvo_simulator_.doStep();

    // Visualize the HRVO Simulator for the current robot
    hrvo_simulator_.visualize(robot_id);

    switch (current_primitive_.primitive_case())
    {
        case TbotsProto::Primitive::kEstop:
        {
            // Protobuf guarantees that the default values in a proto are all zero (bools
            // are false)
            //
            // https://developers.google.com/protocol-buffers/docs/proto3#default
            auto output = std::make_unique<TbotsProto::DirectControlPrimitive>();

            return output;
        }
        case TbotsProto::Primitive::kStop:
        {
            auto prim   = createDirectControlPrimitive(Vector(), AngularVelocity(), 0.0,
                                                     TbotsProto::AutoChipOrKick());
            auto output = std::make_unique<TbotsProto::DirectControlPrimitive>(
                prim->direct_control());
            return output;
        }
        case TbotsProto::Primitive::kDirectControl:
        {
            return std::make_unique<TbotsProto::DirectControlPrimitive>(
                current_primitive_.direct_control());
        }
        case TbotsProto::Primitive::kMove:
        {
            // Compute the target velocities
            Vector target_velocity = getTargetLinearVelocity(current_primitive_.move(), robot_state);
                    //getTargetLinearVelocity(robot_id, robot_state.orientation());
//                        Vector target_velocity = Vector(0,0);

            AngularVelocity target_angular_velocity =
                getTargetAngularVelocity(current_primitive_.move(), robot_state.orientation());

            auto output = createDirectControlPrimitive(
                target_velocity, target_angular_velocity,
                current_primitive_.move().dribbler_speed_rpm(),
                current_primitive_.move().auto_chip_or_kick());

            plotjuggler_values.insert({std::to_string(robot_id) + team_color + "_vt", target_angular_velocity.toRadians()});
            plotjuggler_values.insert({std::to_string(robot_id) + team_color + "_vx", target_velocity.x()});
            plotjuggler_values.insert({std::to_string(robot_id) + team_color + "_vy", target_velocity.y()});
            plotjuggler_values.insert({std::to_string(robot_id) + team_color + "_vxy", target_velocity.length()});
            LOG(PLOTJUGGLER) << *createPlotJugglerValue(plotjuggler_values);
            plotjuggler_values.clear();

            return std::make_unique<TbotsProto::DirectControlPrimitive>(
                output->direct_control());
        }
        case TbotsProto::Primitive::PRIMITIVE_NOT_SET:
        {
            // TODO (#2283) Once we can add/remove robots, this log should
            // be re-enabled. Right now it just gets spammed because we command
            // 6 robots for Div B when there are 11 on the field.
            //
            // LOG(DEBUG) << "No primitive set!";
        }
    }
    return std::make_unique<TbotsProto::DirectControlPrimitive>();
}
