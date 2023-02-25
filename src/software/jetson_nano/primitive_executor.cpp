#include "software/jetson_nano/primitive_executor.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "software/physics/velocity_conversion_util.h"

PrimitiveExecutor::PrimitiveExecutor(const double time_step,
                                     const RobotConstants_t &robot_constants,
                                     const TeamColour friendly_team_colour,
                                     const RobotId robot_id)
    : current_primitive_(),
      robot_constants_(robot_constants),
      hrvo_simulator_(static_cast<float>(time_step), robot_constants,
                      friendly_team_colour),
      time_step_s_(time_step),
      curr_angular_velocity_(AngularVelocity::zero()),
      curr_orientation_(Angle::zero()),
      robot_id_(robot_id),
      curr_local_velocity_(),
      curr_global_position_(),
      friendly_team_colour(friendly_team_colour),
      team_color(friendly_team_colour == TeamColour::YELLOW ? "y" : "b"),
      // TODO: Twente has a different PID constants for their Goalie vs other robots
      // TODO: Should we use I term? If so, we need to reset it to avoid it increasing forever
      angular_speed_pid_(
              robot_constants.robot_max_ang_speed_rad_per_s,
              2.8,
              0.0,
              0.0),
      linear_speed_x_pid_(
              robot_constants.robot_max_speed_m_per_s,
              2.0,
              0,
              0),
      linear_speed_y_pid_(
              robot_constants.robot_max_speed_m_per_s,
              2.0,
              0,
              0),
      last_pos_updated_time(std::chrono::steady_clock::now()),
      enable_velocity_feedback(false)
{
}
// TODO: Tuned PD Simulation constants: X,Y => P=0.3, D=0.13 with 1*max_acceleration (2.1sec)

void PrimitiveExecutor::updatePrimitiveSet(
    const TbotsProto::PrimitiveSet &primitive_set_msg)
{
    hrvo_simulator_.updatePrimitiveSet(primitive_set_msg);
    auto primitive_set_msg_iter = primitive_set_msg.robot_primitives().find(robot_id_);
    if (primitive_set_msg_iter != primitive_set_msg.robot_primitives().end())
    {
        current_primitive_ = primitive_set_msg_iter->second;
        return;
    }
}

void PrimitiveExecutor::setStopPrimitive()
{
    current_primitive_ = *createStopPrimitive();
}

void PrimitiveExecutor::updateWorld(const TbotsProto::World &world_msg)
{
    // TODO: If this value is accurate, we should step simulation by this amount
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_world_network_delay",
                               (createCurrentTimestamp()->epoch_timestamp_seconds() -
                                world_msg.time_sent().epoch_timestamp_seconds()) * MILLISECONDS_PER_SECOND});

    World new_world = World(world_msg);
    hrvo_simulator_.updateWorld(new_world);

    auto this_robot = new_world.friendlyTeam().getRobotById(robot_id_);
    if (this_robot.has_value())
    {
        curr_global_position_ = this_robot->position();
        curr_orientation_ = this_robot->orientation();
        last_pos_updated_time = std::chrono::steady_clock::now();
        plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_x_world", curr_global_position_.x()});
        plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_y_world", curr_global_position_.y()});
        plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_t_world", curr_orientation_.toRadians()});
    }
}

void PrimitiveExecutor::updateVelocity(const Vector &local_velocity,
                                       const AngularVelocity &angular_velocity)
{
    if (enable_velocity_feedback)
    {
        hrvo_simulator_.updateRobotVelocity(
                robot_id_, localToGlobalVelocity(local_velocity, curr_orientation_));
    }

    const auto now = std::chrono::steady_clock::now();
    const double time_since_last_update = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_pos_updated_time).count()) * SECONDS_PER_NANOSECOND;
    last_pos_updated_time = std::chrono::steady_clock::now();

    // Update state
    curr_global_position_ += localToGlobalVelocity((local_velocity + curr_local_velocity_) / 2, curr_orientation_) * time_since_last_update;
    curr_orientation_ += ((angular_velocity + curr_angular_velocity_) / 2) * time_since_last_update;

    curr_angular_velocity_ = angular_velocity;
    curr_local_velocity_ = local_velocity;

    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vx_actual_local", curr_local_velocity_.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vy_actual_local", curr_local_velocity_.y()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vxy_actual_local", curr_local_velocity_.length()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt_actual", curr_angular_velocity_.toRadians()});
}

Vector PrimitiveExecutor::getTargetLinearVelocity()
{
    Vector target_global_velocity = hrvo_simulator_.getRobotVelocity(robot_id_);
    return globalToLocalVelocity(target_global_velocity, curr_orientation_);
}

Vector PrimitiveExecutor::getTargetLinearVelocity(const TbotsProto::MovePrimitive &move_primitive, const Duration time_step)
{
    return Vector(); // TODO: REEEEEMOOOOOOVVVEEEEE

    const Point final_position =
        createPoint(move_primitive.motion_control().path().points().at(1));
    Vector local_distance_delta = globalToLocalVelocity(final_position - curr_global_position_, curr_orientation_);

    const double x = linear_speed_x_pid_.calculate(local_distance_delta.x(), 0.0, time_step.toSeconds(), "x1");
    const double y = linear_speed_y_pid_.calculate(local_distance_delta.y(), 0.0, time_step.toSeconds(), "y1");
    Vector pid_vel = Vector(x, y);
    Vector delta_vel = pid_vel - curr_local_velocity_;

    // Clamp to max acceleration
    float acceleration_limit;
    if (pid_vel.length() >= curr_local_velocity_.length())
    {
        acceleration_limit = robot_constants_.robot_max_acceleration_m_per_s_2;
    }
    else
    {
        acceleration_limit = robot_constants_.robot_max_deceleration_m_per_s_2;
    }
    Vector max_accel = delta_vel.normalize(std::min(delta_vel.length(), acceleration_limit * time_step.toSeconds()));
    Vector desired_output = curr_local_velocity_ + max_accel;

    // Clamp to max speed
    Vector output = desired_output.normalize(std::min(desired_output.length(), static_cast<double>(robot_constants_.robot_max_speed_m_per_s)));

    // Compensate for angular velocity
    output = output.rotate(-curr_angular_velocity_ * time_step.toSeconds() * 0.5);

    // Visualization
    Vector xy_inc_global = localToGlobalVelocity(max_accel, curr_orientation_);
    Vector output_global = localToGlobalVelocity(output, curr_orientation_);
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_x_diff", (final_position - curr_global_position_).x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_y_diff", (final_position - curr_global_position_).y()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_x_diff_local", local_distance_delta.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_y_diff_local", local_distance_delta.y()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_xy_diff", local_distance_delta.length()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_x_est", curr_global_position_.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_y_est", curr_global_position_.y()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_x_inc_local", max_accel.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_y_inc_local", max_accel.y()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_x_pid_diff", (pid_vel - curr_local_velocity_).x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_y_pid_diff", (pid_vel - curr_local_velocity_).y()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_x_inc_global", xy_inc_global.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_y_inc_global", xy_inc_global.y()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vxy_len", output_global.length()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vx", output_global.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vy", output_global.y()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vx_local", output.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vy_local", output.y()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_x_dest", final_position.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_y_dest", final_position.y()});

    Vector v_diff = (output - output.project(globalToLocalVelocity(final_position - curr_global_position_, curr_orientation_)));
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vxy_diff", v_diff.length()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vx_diff", v_diff.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vy_diff", v_diff.y()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_prim_exec_time_step_ms", time_step.toMilliseconds()});

    return output;
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity(
    const TbotsProto::MovePrimitive &move_primitive, const Duration time_step)
{
    const Angle dest_orientation = createAngle(move_primitive.final_angle());
    const double signed_delta_orientation =
            (dest_orientation - curr_orientation_).clamp().toRadians();

    // PID controller
    const double pid_output = angular_speed_pid_.calculate(signed_delta_orientation, 0.0, time_step.toSeconds(), "t");
    AngularVelocity pid_angular_velocity = AngularVelocity::fromRadians(pid_output);

    // Clamp acceleration
    double delta_angular_velocity = (pid_angular_velocity - curr_angular_velocity_).toRadians();
    const double max_accel = robot_constants_.robot_max_ang_acceleration_rad_per_s_2 * time_step.toSeconds();
    const double clamped_delta_angular_velocity = std::clamp(delta_angular_velocity, -max_accel, max_accel);

    // Clamp velocity
    const double desired_output = curr_angular_velocity_.toRadians() + clamped_delta_angular_velocity;
    const double max_angular_vel = static_cast<double>(robot_constants_.robot_max_ang_speed_rad_per_s);
    AngularVelocity output = AngularVelocity::fromRadians(std::clamp(desired_output, -max_angular_vel, max_angular_vel));

    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt_actual", curr_angular_velocity_.toRadians()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt_output", output.toRadians()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt_pid_output", pid_output});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt_pid_delta", delta_angular_velocity});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt_clamped_delta", clamped_delta_angular_velocity});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt_desired_output", desired_output});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_t_desired", dest_orientation.toRadians()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_t_signedDeltaToDest", signed_delta_orientation});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_t_primexec", curr_orientation_.toRadians()});

    return output;
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive(const Duration time_step)
{
    hrvo_simulator_.doStep(curr_orientation_, curr_angular_velocity_);

    // Visualize the HRVO Simulator for the current robot
    hrvo_simulator_.visualize(robot_id_);

    switch (current_primitive_.primitive_case())
    {
        case TbotsProto::Primitive::kStop:
        {
            auto prim   = createDirectControlPrimitive(Vector(), AngularVelocity(), 0.0,
                                                     TbotsProto::AutoChipOrKick());
            auto output = std::make_unique<TbotsProto::DirectControlPrimitive>(
                prim->direct_control());
            enable_velocity_feedback = true;
            return output;
        }
        case TbotsProto::Primitive::kDirectControl:
        {
            return std::make_unique<TbotsProto::DirectControlPrimitive>(
                current_primitive_.direct_control());
        }
        case TbotsProto::Primitive::kMove:
        {
            enable_velocity_feedback = false;
            // Compute the target velocities
//            Vector target_velocity = getTargetLinearVelocity(current_primitive_.move(), time_step); // PID
            Vector target_velocity = getTargetLinearVelocity(); // HRVO
//            target_velocity = globalToLocalVelocity(Vector(0, -1), curr_orientation_); // TODO: See if robot actually moves down while rotating
//              Vector target_velocity = getTargetLinearVelocity(robot_id, robot_state.orientation());

            AngularVelocity target_angular_velocity = getTargetAngularVelocity(current_primitive_.move(), time_step);

//            target_velocity = Vector(target_velocity.x(), 0.0);
//            target_velocity = Vector();
//            target_angular_velocity = AngularVelocity::fromDegrees(0.0);

            auto output = createDirectControlPrimitive(
                target_velocity, target_angular_velocity,
                current_primitive_.move().dribbler_speed_rpm(),
                current_primitive_.move().auto_chip_or_kick());

            plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt", target_angular_velocity.toRadians()});
            plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vx_desired", target_velocity.x()});
            plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vy_desired", target_velocity.y()});
            plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vxy_desired", target_velocity.length()});
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

void PrimitiveExecutor::setRobotId(const RobotId robot_id)
{
    robot_id_ = robot_id;
}
