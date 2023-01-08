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
      angular_speed_pid_(time_step,
                       robot_constants.robot_max_ang_acceleration_rad_per_s_2 * time_step,
                       -robot_constants.robot_max_ang_acceleration_rad_per_s_2 * time_step,
                         0.8, // 0.3, 0.15, 0.0
                         0.16,
                         0.0),
      linear_speed_x_pid_(time_step,
                         6 * time_step, //robot_constants.robot_max_acceleration_m_per_s_2
                       -6 * time_step,
                       0.3,
                       0.178,
                       0),
      linear_speed_y_pid_(time_step,
                          6 * time_step,
                          -6 * time_step,
                          0.3,
                          0.14,
                          0)
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
    World new_world = World(world_msg);
    hrvo_simulator_.updateWorld(new_world);

    auto this_robot = new_world.friendlyTeam().getRobotById(robot_id_);
    if (this_robot.has_value())
    {
        curr_global_position_ = this_robot->position();
        curr_orientation_ = this_robot->orientation();
    }
}

void PrimitiveExecutor::updateVelocity(const Vector &local_velocity,
                                       const AngularVelocity &angular_velocity)
{
    hrvo_simulator_.updateRobotVelocity(
            robot_id_, localToGlobalVelocity(local_velocity, curr_orientation_));
    curr_angular_velocity_ = angular_velocity;
    curr_local_velocity_ = local_velocity;
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vx_actual_local", curr_local_velocity_.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vy_actual_local", curr_local_velocity_.y()});
}

Vector PrimitiveExecutor::getTargetLinearVelocity()
{
    Vector target_global_velocity = hrvo_simulator_.getRobotVelocity(robot_id_);
    return globalToLocalVelocity(target_global_velocity, curr_orientation_);
}

Vector PrimitiveExecutor::getTargetLinearVelocity(const TbotsProto::MovePrimitive &move_primitive)
{
    // TODO: I wonder if HRVO will also oscillate if we used local velocity (takes into account the changing orientation)
    //       instead of global velocity
    const Point final_position =
        createPoint(move_primitive.motion_control().path().points().at(1));
    Vector local_distance_delta = globalToLocalVelocity(final_position - curr_global_position_, curr_orientation_);

    const double x_inc = linear_speed_x_pid_.calculate(local_distance_delta.x(), 0.0);
    const double y_inc = linear_speed_y_pid_.calculate(local_distance_delta.y(), 0.0);

    Vector output = curr_local_velocity_ + Vector(x_inc, y_inc);
    output = Vector(output.x(), output.y());

    Vector xy_inc_global = localToGlobalVelocity(Vector(x_inc, y_inc), curr_orientation_);
    Vector output_global = localToGlobalVelocity(output, curr_orientation_);
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_x_diff", (final_position - curr_global_position_).x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_y_diff", (final_position - curr_global_position_).y()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_x_est", curr_global_position_.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_y_est", curr_global_position_.y()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_x_inc_local", x_inc});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_y_inc_local", y_inc});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_x_inc_global", xy_inc_global.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_y_inc_global", xy_inc_global.y()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vxy_len", output_global.length()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vx", output_global.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vy", output_global.y()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vx_local", output.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vy_local", output.y()});
//    Vector curr_global_velocity = localToGlobalVelocity(curr_local_velocity_, curr_orientation_);
//    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vx_actual", curr_global_velocity.x()});
//    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vy_actual", curr_global_velocity.y()});

    Vector v_diff = (output - output.project(globalToLocalVelocity(final_position - curr_global_position_, curr_orientation_)));
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vxy_diff", v_diff.length()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vx_diff", v_diff.x()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vy_diff", v_diff.y()});


//    LOG(VISUALIZE) << *createSegmentProto(Segment(curr_global_position_,
//                                                  curr_global_position_ + Vector(x_inc, 0.0)));
//    LOG(VISUALIZE) << *createSegmentProto(Segment(curr_global_position_,
//                                                  curr_global_position_ + Vector(0.0, y_inc)));

    // Update estimated local velocity
    curr_local_velocity_ = output;
    // TODO: Should be set to actual position everytime a new world is received!!
    // TODO: Use robotState instead of curr_global_position_
    curr_global_position_ += output_global * time_step_s_;
    return output;
}

AngularVelocity PrimitiveExecutor::getTargetAngularVelocity(
    const TbotsProto::MovePrimitive &move_primitive)
{
    const Angle dest_orientation = createAngle(move_primitive.final_angle());
    const double signed_delta_orientation =
            (dest_orientation - curr_orientation_).clamp().toRadians();

    // TODO: Should we be using feedback from the robot here: curr_angular_velocity_?
    const double inc = angular_speed_pid_.calculate(signed_delta_orientation, 0.0);
    AngularVelocity output = AngularVelocity::fromRadians(curr_angular_velocity_.toRadians() + inc);

    // Used to stop Jitter when at destination
    // Value determined experimentally
//    if (abs(signed_delta_orientation) < Angle::fromDegrees(2).toRadians())
//    {
//        output = AngularVelocity::zero();
//    }

    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt_actual", curr_angular_velocity_.toRadians()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt_output", output.toRadians()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt_pid_inc", inc});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_t_desired", dest_orientation.toRadians()});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_t_signedDeltaToDest", signed_delta_orientation});
    plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_t_primexec", curr_orientation_.toRadians()});

    curr_angular_velocity_ = output;
    // TODO: Should be set to actual position everytime a new world is received!!
    // TODO: Use robotState instead of curr_orientation_
    curr_orientation_ += curr_angular_velocity_ * time_step_s_;
    // TODO: Nima remove support for turning
    return output;
}


std::unique_ptr<TbotsProto::DirectControlPrimitive> PrimitiveExecutor::stepPrimitive()
{
    hrvo_simulator_.doStep();

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
            Vector target_velocity = getTargetLinearVelocity(current_primitive_.move());
//            target_velocity = Vector(0, -1).rotate(-curr_orientation_); // TODO: See if robot actually moves down while rotating
//              Vector target_velocity = getTargetLinearVelocity(robot_id, robot_state.orientation());

            AngularVelocity target_angular_velocity = getTargetAngularVelocity(current_primitive_.move());

//            target_velocity = Vector(target_velocity.x(), 0.0);
//            target_velocity = Vector();
//            target_angular_velocity = AngularVelocity::fromDegrees(60.0);

            auto output = createDirectControlPrimitive(
                target_velocity, target_angular_velocity,
                current_primitive_.move().dribbler_speed_rpm(),
                current_primitive_.move().auto_chip_or_kick());

            plotjuggler_values.insert({std::to_string(robot_id_) + team_color + "_vt", target_angular_velocity.toRadians()});
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
