#include "software/jetson_nano/primitive_executor.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/math/math_functions.h"

PrimitiveExecutor::PrimitiveExecutor(const double time_step,
                                     const RobotConstants_t& robot_constants,
                                     const TeamColour friendly_team_colour)
    : current_primitive_(),
      robot_constants_(robot_constants),
      hrvo_simulator_(static_cast<float>(time_step), robot_constants,
                      friendly_team_colour),
      time_step_s_(time_step),
      prev_linear_wheel_velocities(WheelSpace_t::Zero()),
      prev_angular_wheel_velocities(WheelSpace_t::Zero()),
      euclidean_to_four_wheel(robot_constants)
{
    LOG(DEBUG) << "Primitive Executor running with time step = " << time_step;
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
    // const float LOCAL_EPSILON = 1e-6f;  // Avoid dividing by zero

    //// Unpack current move primitive
    //const float dest_linear_speed = move_primitive.final_speed_m_per_s();
    //const float max_speed_m_per_s = moveu_primitive.max_speed_m_per_s();
    const Point final_position =
        createPoint(move_primitive.motion_control().path().points().at(1));

    //const float max_target_linear_speed = fmaxf(max_speed_m_per_s, dest_linear_speed);

    //// Compute distance to destination
    const float norm_dist_delta =
        static_cast<float>((robot_state.position() - final_position).length());

    //// Compute at what linear distance we should start decelerating
    //// d = (Vf^2 - Vi^2) / (2a + LOCAL_EPSILON)
    //const float start_linear_deceleration_distance =
        //(max_target_linear_speed * max_target_linear_speed -
         //dest_linear_speed * dest_linear_speed) /
        //(2 * move_primitive.robot_max_acceleration_m_per_s_2() + LOCAL_EPSILON);

////    (max_target_linear_speed * max_target_linear_speed) /  // Changes formula here and removed dest_linear_speed * dest_linear_speed
////    (2 * move_primitive.robot_max_acceleration_m_per_s_2() + LOCAL_EPSILON)

    //// When we are close enough to start decelerating, we reduce the max speed
    //// by 60%. Once we get closer than 0.6 meters, we start to linearly decrease
    //// speed proportional to the distance to the destination. 0.6 was determined
    //// experimentally.
    //float target_linear_speed = max_target_linear_speed;
    //if (norm_dist_delta < start_linear_deceleration_distance)
    //{
        //target_linear_speed = max_target_linear_speed * fminf(norm_dist_delta, 0.6f);
    //}

    Vector target_global_velocity = final_position - robot_state.position();

    double local_x_velocity =
        robot_state.orientation().cos() * target_global_velocity.x() +
        robot_state.orientation().sin() * target_global_velocity.y();

    double local_y_velocity =
        -robot_state.orientation().sin() * target_global_velocity.x() +
        robot_state.orientation().cos() * target_global_velocity.y();

    return Vector(local_x_velocity, local_y_velocity).normalize(norm_dist_delta * 2.5f);
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity(
    const TbotsProto::MovePrimitive& move_primitive, const Angle& curr_orientation)
{
    const Angle dest_orientation = createAngle(move_primitive.final_angle());
    const double delta_orientation =
        dest_orientation.minDiff(curr_orientation).toRadians();

    // The speed which we should be decelerating at to stop at the destination,
    // derived by solving for v_i in the equation v_f^2 = v_i^2 + 2*a*d.
    double acceleration_angular_speed =
        curr_angular_velocity_.toRadians() +
        robot_constants_.robot_max_ang_acceleration_rad_per_s_2 * time_step_s_;
    double deceleration_angular_speed = std::sqrt(
        2 * robot_constants_.robot_max_ang_acceleration_rad_per_s_2 * delta_orientation);
    double max_angular_speed =
        static_cast<double>(robot_constants_.robot_max_ang_speed_rad_per_s);
    double next_angular_speed = std::min(
        {max_angular_speed, deceleration_angular_speed, acceleration_angular_speed});

    const double signed_delta_orientation =
        (dest_orientation - curr_orientation).clamp().toRadians();
    std::cout << "delta_orientation=" << dest_orientation - curr_orientation << " next_angular_speed=" << AngularVelocity::fromRadians(
            std::copysign(next_angular_speed, signed_delta_orientation)).toRadians() << std::endl;
    return AngularVelocity::fromRadians(
        std::copysign(next_angular_speed, signed_delta_orientation));
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(
    const unsigned int robot_id,
    const RobotState& robot_state)  // TODO Nima: Revert to angle (orientation)
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
            Vector target_velocity =
                getTargetLinearVelocity(current_primitive_.move(), robot_state);
            //            Vector target_velocity =
            //            getTargetLinearVelocity(current_primitive_.move(), robot_state);
            AngularVelocity target_angular_velocity = getTargetAngularVelocity(
                current_primitive_.move(), robot_state.orientation());

//            auto [ramped_target_velocity, ramped_target_angular_velocity] =
//                rampVelocity(target_velocity, target_angular_velocity);

//            ramped_target_velocity         = target_velocity;
//            ramped_target_angular_velocity = target_angular_velocity;

            auto output = createDirectControlPrimitive(
                    target_velocity, target_angular_velocity,
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

std::pair<Vector, AngularVelocity> PrimitiveExecutor::rampVelocity(
    const Vector& vector, const AngularVelocity& angle)
{
    // TODO: This code should be in erforcesimulator.cpp and should be shared with
    // motor.cpp (not copied)
    // TODO #1: Make ErForce use infinite acceleration
    // Convert to euclidean
//    EuclideanSpace_t target_velocity = {-vector.y(), vector.x(), angle.toRadians()};

//    WheelSpace_t target_linear_wheel_velocities = rampWheelVelocity(
//        prev_linear_wheel_velocities, target_velocity,
//        static_cast<double>(robot_constants_.robot_max_speed_m_per_s),
//        static_cast<double>(robot_constants_.robot_max_acceleration_m_per_s_2),
//        time_step_s_);

    //    WheelSpace_t target_linear_wheel_velocities = rampWheelVelocity(
    //            prev_linear_wheel_velocities, {target_velocity[0], target_velocity[1],
    //            0.0}, static_cast<double>(robot_constants_.robot_max_speed_m_per_s),
    //            static_cast<double>(robot_constants_.robot_max_acceleration_m_per_s_2),
    //            time_step_s_);
    //
    //    WheelSpace_t target_angular_wheel_velocities = rampWheelVelocity(
    //            prev_angular_wheel_velocities, {0.0, 0.0, target_velocity[2]},
    //            static_cast<double>(robot_constants_.robot_max_ang_speed_rad_per_s
    //            / 5.0),
    //            static_cast<double>(robot_constants_.robot_max_ang_acceleration_rad_per_s_2
    //            / 5.0), time_step_s_);

//    prev_linear_wheel_velocities = target_linear_wheel_velocities;
    //    prev_angular_wheel_velocities = target_angular_wheel_velocities;
    //    WheelSpace_t target_total_wheel_velocities =
    //            prev_linear_wheel_velocities + prev_angular_wheel_velocities;
//    WheelSpace_t target_total_wheel_velocities = prev_linear_wheel_velocities;
//
//    EuclideanSpace_t ramped_euclidean_velocity =
//        euclidean_to_four_wheel.getEuclideanVelocity(target_total_wheel_velocities);

    // Convert back
//    Vector ramped_linear_velocity(ramped_euclidean_velocity[1],
//                                  -ramped_euclidean_velocity[0]);
//    AngularVelocity ramped_angular_velocity =
//        AngularVelocity::fromRadians(ramped_euclidean_velocity[2]);
    return {Vector(), AngularVelocity::fromDegrees(0)};
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
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);

    // Ramp wheel velocity vector
    // Step 1: Find absolute max velocity delta
    auto delta_target_wheel_velocity = target_wheel_velocity - current_wheel_velocity;
    auto max_delta_target_wheel_velocity =
        delta_target_wheel_velocity.cwiseAbs().maxCoeff();

    // Step 2: Compare max delta velocity against the calculated maximum
    if (max_delta_target_wheel_velocity > allowable_delta_wheel_velocity)
    {
        // Step 3: If larger, scale down to allowable max
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
        // if larger, scale down to max
        ramp_wheel_velocity = (ramp_wheel_velocity / max_ramp_wheel_velocity) *
                              max_allowable_wheel_velocity;
    }

    return ramp_wheel_velocity;
}
