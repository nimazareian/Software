#include "software/jetson_nano/primitive_executor.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/logger/logger.h"
#include "software/math/math_functions.h"
#include "proto/message_translation/tbots_protobuf.h"

PrimitiveExecutor::PrimitiveExecutor(const double time_step, const RobotId robot_id,
                                     const RobotConstants_t &robot_constants,
                                     const TeamColour friendly_team_colour)
    : robot_id_(robot_id),
      time_step_(time_step),
      current_primitive_(),
      robot_constants_(robot_constants),
      hrvo_simulator_(static_cast<float>(time_step), robot_constants,
                      friendly_team_colour),
      euclidean_to_four_wheel_(robot_constants),
      prev_linear_wheel_velocities(WheelSpace_t::Zero()),
      prev_angular_wheel_velocities(WheelSpace_t::Zero()),
      prev_wheel_velocities(WheelSpace_t::Zero())
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
    }
}

void PrimitiveExecutor::updateWorld(const TbotsProto::World& world_msg)
{
    hrvo_simulator_.updateWorld(World(world_msg));
}

void PrimitiveExecutor::updateLocalVelocity(const Vector &local_velocity,
                                            const Angle &curr_orientation) {
    // instead of wheel accel being scaled down 84 times, it only happened 12
    if (++frame % 30 == 0) {
        hrvo_simulator_.updateFriendlyRobotVelocity(robot_id_,
                                                    local_velocity.rotate(-curr_orientation));
        prev_local_velocity_ = local_velocity;
        frame = 0;
        std::cout << local_velocity.length() << " :: local velocity" << std::endl;
        std::cout << (local_velocity - prev_set_velocity_).length() / time_step_ << " :: delta prim exec velocity (local - prev set vel = " << local_velocity - prev_set_velocity_ << std::endl;
    }
    else
    {
        std::cout << "not updating velocity" << std::endl;
    }
}

// TODO: Replace robot_id with robot_id_ member variable
Vector PrimitiveExecutor::getTargetLinearVelocity(const unsigned int robot_id,
                                                  const Angle& curr_orientation)
{
    Vector target_global_velocity = hrvo_simulator_.getRobotVelocity(robot_id);
    return target_global_velocity.rotate(-curr_orientation);
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity(
    const TbotsProto::MovePrimitive& move_primitive, const Angle& curr_orientation)
{
    const Angle dest_orientation = createAngle(move_primitive.final_angle());
    const double delta_orientation =
        dest_orientation.minDiff(curr_orientation).toRadians();

    // angular velocity given linear deceleration and distance remaining to target
    // orientation.
    // Vi = sqrt(0^2 + 2 * a * d)
    double deceleration_angular_speed = std::sqrt(
        2 * robot_constants_.robot_max_ang_acceleration_rad_per_s_2 * delta_orientation);

    double max_angular_speed =
        static_cast<double>(robot_constants_.robot_max_ang_speed_rad_per_s);
    double next_angular_speed = std::min(max_angular_speed, deceleration_angular_speed);

    const double signed_delta_orientation =
        (dest_orientation - curr_orientation).clamp().toRadians();
    return AngularVelocity::fromRadians(
        std::copysign(next_angular_speed, signed_delta_orientation));
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(
    const unsigned int robot_id, const Angle& curr_orientation)
{
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

            // Discharge the capacitors
            output->mutable_power_control()->set_charge_mode(
                TbotsProto::PowerControl_ChargeMode_DISCHARGE);

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
            Vector target_linear_velocity =
                getTargetLinearVelocity(robot_id, curr_orientation);
            AngularVelocity target_angular_velocity =
                getTargetAngularVelocity(current_primitive_.move(), curr_orientation);

            // TODO: Create a struct for WheelSpace_t and EuclideanSpace_t
            EuclideanSpace_t target_euclidean_velocity = {
                target_linear_velocity.x(), target_linear_velocity.y(),
                target_angular_velocity.toRadians()};

            WheelSpace_t target_linear_wheel_velocities = rampWheelVelocity(
                prev_linear_wheel_velocities, {target_euclidean_velocity[0], target_euclidean_velocity[1], 0.0},
                static_cast<double>(robot_constants_.robot_max_speed_m_per_s/* + 100.f*/),
                static_cast<double>(robot_constants_.robot_max_acceleration_m_per_s_2/* + 200.f*/),
                time_step_);
            prev_linear_wheel_velocities = target_linear_wheel_velocities;

            WheelSpace_t target_angular_wheel_velocities =
                        rampWheelVelocity(
                                prev_angular_wheel_velocities, {0.0, 0.0,
                                target_euclidean_velocity[2]},
                                static_cast<double>(robot_constants_.robot_max_ang_speed_rad_per_s /*+ 150.f*/),
                                static_cast<double>(robot_constants_.robot_max_ang_acceleration_rad_per_s_2/* + 100.f*/),
                                time_step_);
            prev_angular_wheel_velocities = target_angular_wheel_velocities;

//            WheelSpace_t target_total_wheel_velocities =
//                        rampWheelVelocity(
//                                prev_linear_wheel_velocities, target_euclidean_velocity,
//                                static_cast<double>(robot_constants_.robot_max_speed_m_per_s),
//                                static_cast<double>(robot_constants_.robot_max_acceleration_m_per_s_2),
//                                time_step_);
//            prev_linear_wheel_velocities = target_total_wheel_velocities;//target_angular_wheel_velocities;

            WheelSpace_t target_total_wheel_velocities =
                                target_linear_wheel_velocities +
                                target_angular_wheel_velocities;

//            WheelSpace_t target_total_wheel_velocities =
//                    euclidean_to_four_wheel_.getWheelVelocity(
//                            target_euclidean_velocity);

            // convert euclidean to wheel velocity
            EuclideanSpace_t ramped_euclidean_velocity =
                euclidean_to_four_wheel_.getEuclideanVelocity(
                    target_total_wheel_velocities);
            //


            // TODO: Create a struct for WheelSpace_t and EuclideanSpace_t
            //            EuclideanSpace_t target_euclidean_velocity =
            //            {target_linear_velocity.x(),
            //                                                          target_linear_velocity.y(),
            //
            //                                                          target_angular_velocity.toRadians()};
            //            // convert euclidean to wheel velocity
            //            WheelSpace_t target_wheel_velocity =
            //                    euclidean_to_four_wheel_.getWheelVelocity(target_euclidean_velocity);
            //            EuclideanSpace_t ramped_euclidean_velocity =
            //            euclidean_to_four_wheel_.getEuclideanVelocity(target_wheel_velocity);
            ////
            //////            // TODO: Might have to have the x y  order different


//            LOG(VISUALIZE) << *createNamedValue(
//                        "wheel 0 vel",
//                        static_cast<float>(target_total_wheel_velocities[0]));
//            LOG(VISUALIZE) << *createNamedValue(
//                        "wheel 1 vel",
//                        static_cast<float>(target_total_wheel_velocities[1]));
//            LOG(VISUALIZE) << *createNamedValue(
//                        "wheel 2 vel",
//                        static_cast<float>(target_total_wheel_velocities[2]));
//            LOG(VISUALIZE) << *createNamedValue(
//                        "wheel 3 vel",
//                        static_cast<float>(target_total_wheel_velocities[3]));

//            WheelSpace_t wheel_accel = target_total_wheel_velocities - prev_wheel_velocities;
            prev_wheel_velocities = target_total_wheel_velocities;
//            LOG(VISUALIZE) << *createNamedValue(
//                        "wheel 0 accel",
//                        static_cast<float>(wheel_accel[0] / time_step_));
//            LOG(VISUALIZE) << *createNamedValue(
//                        "wheel 1 accel",
//                        static_cast<float>(wheel_accel[1] / time_step_));
//            LOG(VISUALIZE) << *createNamedValue(
//                        "wheel 2 accel",
//                        static_cast<float>(wheel_accel[2] / time_step_));
//            LOG(VISUALIZE) << *createNamedValue(
//                        "wheel 3 accel",
//                        static_cast<float>(wheel_accel[3] / time_step_));

            target_linear_velocity =
                Vector(ramped_euclidean_velocity[0], ramped_euclidean_velocity[1]);
            target_angular_velocity =
                AngularVelocity::fromRadians(ramped_euclidean_velocity[2]);

            prev_set_velocity_ = target_linear_velocity;
            std::cout << target_linear_velocity.length()  << " : vel in prim exec to set =>" << (target_linear_velocity - prev_local_velocity_).length() / time_step_ << " : accel prim exec / sec" << std::endl;

            auto output = createDirectControlPrimitive(
                target_linear_velocity, target_angular_velocity,
                current_primitive_.move().dribbler_speed_rpm(),
                current_primitive_.move().auto_chip_or_kick());

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

WheelSpace_t PrimitiveExecutor::rampWheelVelocity(
    const WheelSpace_t& current_wheel_velocity,
    const EuclideanSpace_t& target_euclidean_velocity,
    double max_allowable_wheel_velocity, double allowed_acceleration,
    const double& time_to_ramp)
{
    // ramp wheel velocity
    WheelSpace_t ramp_wheel_velocity;

    // calculate max allowable wheel velocity delta using dv = a*t
    auto allowable_delta_wheel_velocity = allowed_acceleration * time_to_ramp;

    // convert euclidean to wheel velocity
    WheelSpace_t target_wheel_velocity =
        euclidean_to_four_wheel_.getWheelVelocity(target_euclidean_velocity);

    // Ramp wheel velocity vector
    // Step 1: Find absolute max velocity delta
    auto delta_target_wheel_velocity = target_wheel_velocity - current_wheel_velocity;
    auto max_delta_target_wheel_velocity =
        delta_target_wheel_velocity.cwiseAbs().maxCoeff();

//    LOG(VISUALIZE) << *createNamedValue(
//                "max_wheel_accel",
//                static_cast<float>(max_delta_target_wheel_velocity / time_to_ramp));

    // Step 2: Compare max delta velocity against the calculated maximum
    if (max_delta_target_wheel_velocity > allowable_delta_wheel_velocity)
    {
        // Step 3: If larger, scale down to allowable max
        std::cout << "Scaling down wheel acceleration" << std::endl;
        ramp_wheel_velocity =
            (delta_target_wheel_velocity / max_delta_target_wheel_velocity) *
                allowable_delta_wheel_velocity +
            current_wheel_velocity;
    }
    else
    {
        // If smaller, go straight to target
        ramp_wheel_velocity = target_wheel_velocity;
    }

    // find absolute max wheel velocity
    auto max_ramp_wheel_velocity = ramp_wheel_velocity.cwiseAbs().maxCoeff();

    // compare against max wheel velocity
    if (max_ramp_wheel_velocity > max_allowable_wheel_velocity)
    {
        std::cout << "Scaling down wheel velocity" << std::endl;
        // if larger, scale down to max
        ramp_wheel_velocity = (ramp_wheel_velocity / max_ramp_wheel_velocity) *
                              max_allowable_wheel_velocity;
    }

    return ramp_wheel_velocity;
}

void PrimitiveExecutor::setRobotId(RobotId robot_id)
{
    robot_id_ = robot_id;
}
