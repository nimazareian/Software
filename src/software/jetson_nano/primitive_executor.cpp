#include "software/jetson_nano/primitive_executor.h"

#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "software/logger/logger.h"
#include "software/math/math_functions.h"


void PrimitiveExecutor::startPrimitive(const RobotConstants_t& robot_constants,
                                       const TbotsProto::Primitive& primitive)
{
    robot_constants_   = robot_constants;
    current_primitive_ = primitive;
}

Vector PrimitiveExecutor::getTargetLinearVelocity(
        const TbotsProto::MovePrimitive& move_primitive, unsigned int robot_id) const
{
    // TODO: Get current robot velocity from HRVO sim using the robot_id

    Vector target_global_velocity = Vector();

    double local_x_velocity =
            robot_id.orientation().cos() * target_global_velocity.x() +
            robot_id.orientation().sin() * target_global_velocity.y();

    double local_y_velocity =
            -robot_id.orientation().sin() * target_global_velocity.x() +
            robot_id.orientation().cos() * target_global_velocity.y();

    return Vector(local_x_velocity, local_y_velocity).normalize(target_linear_speed);
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity(
        const TbotsProto::MovePrimitive& move_primitive, unsigned int robot_id)
{
    // TODO: replace float with double
    const float LOCAL_EPSILON = 1e-6f;  // Avoid dividing by zero


    const float dest_orientation =
        static_cast<float>(move_primitive.final_angle().radians());
    const float delta_orientation =
        dest_orientation - static_cast<float>(robot_id.orientation().toRadians());
    const float max_target_angular_speed = robot_constants_.robot_max_ang_speed_rad_per_s;

    // Compute at what angular distance we should start decelerating angularly
    // d = (Vf^2 - 0) / (2a + LOCAL_EPSILON)
    const float start_angular_deceleration_distance =
        (max_target_angular_speed * max_target_angular_speed) /
        (2 * robot_constants_.robot_max_ang_acceleration_rad_per_s_2 + LOCAL_EPSILON);

    const float target_angular_speed =
        max_target_angular_speed *
        static_cast<float>(sigmoid(fabsf(delta_orientation),
                                   start_angular_deceleration_distance / 2,
                                   start_angular_deceleration_distance));

    return AngularVelocity::fromRadians(
        copysign(target_angular_speed, delta_orientation));
}


std::unique_ptr<TbotsProto::DirectControlPrimitive>
PrimitiveExecutor::stepPrimitive(const World &world, unsigned int robot_id)
{
    switch (current_primitive_.primitive_case())
    {
        case TbotsProto::Primitive::kEstop:
        {
            // Protobuf guarantees that the default values in a proto are all zero (bools
            // are false)
            //
            // https://developers.google.com/protocol-buffers/docs/proto3#default
            auto output = std::make_unique<TbotsProto::DirectControlPrimitive>();

            // Discharge the capacitors
            output->set_charge_mode(
                TbotsProto::DirectControlPrimitive_ChargeMode_DISCHARGE);

            return output;
        }
        case TbotsProto::Primitive::kStop:
        {
            auto prim   = createDirectControlPrimitive(Vector(), AngularVelocity(), 0.0);
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
            const std::vector<Robot>& friendly_robots = world.friendlyTeam().getAllRobots();
            auto robot_iter =
                    std::find_if(friendly_robots.begin(), friendly_robots.end(),
                                 [simulator_robot](const Robot& robot) { return robot.id() == simulator_robot->getRobotId(); });
            if (robot_iter != friendly_robots.end())
            {
            // Compute the target velocities
            Vector target_velocity =
                getTargetLinearVelocity(current_primitive_.move(), robot_id);
            AngularVelocity target_angular_velocity =
                getTargetAngularVelocity(current_primitive_.move(), robot_id);

            auto output = createDirectControlPrimitive(
                target_velocity, target_angular_velocity,
                current_primitive_.move().dribbler_speed_rpm());

            // Copy the AutoKickOrChip settings over
            copyAutoChipOrKick(current_primitive_.move(),
                               output->mutable_direct_control());

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

void PrimitiveExecutor::copyAutoChipOrKick(const TbotsProto::MovePrimitive& src,
                                           TbotsProto::DirectControlPrimitive* dest)
{
    switch (src.auto_chip_or_kick().auto_chip_or_kick_case())
    {
        case TbotsProto::MovePrimitive::AutoChipOrKick::AutoChipOrKickCase::
            kAutokickSpeedMPerS:
        {
            dest->set_autokick_speed_m_per_s(
                src.auto_chip_or_kick().autokick_speed_m_per_s());

            break;
        }
        case TbotsProto::MovePrimitive::AutoChipOrKick::AutoChipOrKickCase::
            kAutochipDistanceMeters:
        {
            dest->set_autochip_distance_meters(
                src.auto_chip_or_kick().autochip_distance_meters());
            break;
        }
        case TbotsProto::MovePrimitive::AutoChipOrKick::AutoChipOrKickCase::
            AUTO_CHIP_OR_KICK_NOT_SET:
        {
            dest->clear_chick_command();
            break;
        }
    }
}
