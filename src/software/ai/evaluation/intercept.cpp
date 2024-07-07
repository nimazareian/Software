#include "software/ai/evaluation/intercept.h"

#include "shared/constants.h"
#include "software/ai/evaluation/time_to_travel.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/optimization/gradient_descent_optimizer.hpp"

std::optional<std::pair<Point, Duration>> findBestInterceptForBall(const Ball &ball,
                                                                   const Field &field,
                                                                   const Robot &robot)
{
    static const double gradient_approx_step_size = 0.000001;

    // We use this to take a smooth absolute value in our objective function
    static const double smooth_abs_eps = 1000 * gradient_approx_step_size;

    // This is the objective function that we want to minimize, finding the
    // shortest duration in the future at which we can feasibly intercept the
    // ball
    auto objective_function = [&](std::array<double, 1> x)
    {
        // We take the absolute value here because a negative time makes no sense
        double duration = std::abs(x.at(0));

        // If the ball timestamp is less then the robot timestamp, add the difference
        // here so that we're optimizing to a duration that is after the robot
        // timestamp
        if (ball.timestamp() < robot.timestamp())
        {
            duration += (robot.timestamp() - ball.timestamp()).toSeconds();
        }

        // Estimate the ball position
        Point new_ball_pos =
            ball.estimateFutureState(Duration::fromSeconds(duration)).position();

        // Figure out how long it will take the robot to get to the new ball position
        Duration time_to_ball_pos = robot.getTimeToPosition(new_ball_pos);

        // Figure out when the robot will reach the new ball position relative to the
        // time that the ball will get there (ie. will we get there in time?)
        double ball_robot_time_diff = duration - time_to_ball_pos.toSeconds();

        // We want to get to the ball at the earliest opportunity possible, so
        // aim for a time diff of zero. We use a smooth approximation of
        // the maximum here
        return std::sqrt(std::pow(ball_robot_time_diff, 2) + smooth_abs_eps);
    };

    // Figure out when/where to intercept the ball. We do this by optimizing over
    // the ball position as a function of it's travel time
    // We make the weight here an inverse of the ball speed, so that the gradient
    // descent takes smaller steps when the ball is moving faster
    double descent_weight = 1 / (std::exp(ball.currentState().velocity().length() * 0.5));
    GradientDescentOptimizer<1> optimizer({descent_weight}, gradient_approx_step_size);
    Duration best_ball_travel_duration = Duration::fromSeconds(
        std::abs(optimizer.minimize(objective_function, {0}, 50).at(0)));

    // In the objective function above, if the robot timestamp > ball timestamp, we
    // add on the difference so we get a intercept time after the robot timestamp, so
    // we need to do the same here to get the duration we actually optimized on
    if (robot.timestamp() > ball.timestamp())
    {
        best_ball_travel_duration =
            best_ball_travel_duration + (robot.timestamp() - ball.timestamp());
    }

    Point best_ball_intercept_pos =
        ball.estimateFutureState(best_ball_travel_duration).position();

    // Check that we can get to the best position in time
    Duration time_to_ball_pos     = robot.getTimeToPosition(best_ball_intercept_pos);
    Duration ball_robot_time_diff = time_to_ball_pos - best_ball_travel_duration;
    // NOTE: if ball velocity is 0 then ball travel duration is infinite, so this
    // check isn't relevant in that case
    if (ball.currentState().velocity().length() != 0 &&
        std::abs(ball_robot_time_diff.toSeconds()) > descent_weight)
    {
        return std::nullopt;
    }

    // Check that the best intercept position is actually on the field
    if (!contains(field.fieldLines(), best_ball_intercept_pos))
    {
        return std::nullopt;
    }

    return std::make_pair(best_ball_intercept_pos, time_to_ball_pos);
}

Point findInterceptionPoint(const Robot &robot, const Ball &ball, const Field &field,
                            double ball_moving_slow_speed_threshold,
                            double intercept_position_search_interval)
{
    if (ball.velocity().length() < ball_moving_slow_speed_threshold)
    {
        auto face_ball_vector = (ball.position() - robot.position());
        // Draw a long polygon from the center of robot forward, with the width of the
        // dribbler that the ball should be within before we try to dribble into it.
        float dribbler_width      = robot.robotConstants().dribbler_width_meters;
        Vector robot_front_vector = Vector::createFromAngle(robot.orientation());
        Polygon infront_of_dribbler_polygon = Polygon::fromSegment(
                Segment(robot.position(),
                        robot.position() + robot_front_vector.normalize(20)),
                0.0, dribbler_width / 2.0);

        bool dribbler_aligned_with_ball =
                contains(infront_of_dribbler_polygon, ball.position());
        bool robot_turning_too_fast =
                robot.angularVelocity().toDegrees() >
                40.0;

        double offset_to_ball = 0.0;
        if (!dribbler_aligned_with_ball || robot_turning_too_fast)
        {
            // The ball is not infront of the robot, or the robot is turning too fast
            // so add some additional offset to the ball destination, so we don't bump
            // into it.
            offset_to_ball = 0.08; // TODO (NIMA): Some of these constants were manually copied from the updated DribbleTacticConfig from #3234
        }

        auto point_in_front_of_ball = robotPositionToFaceBall(
                ball.position(), face_ball_vector.orientation(), offset_to_ball);
        return point_in_front_of_ball;
    }

    Point intercept_position = ball.position();
    while (contains(field.fieldLines(), intercept_position))
    {
        Duration ball_time_to_pos = Duration::fromSeconds(
            distance(intercept_position, ball.position()) / ball.velocity().length());
        Duration robot_time_to_pos = robot.getTimeToPosition(intercept_position);

        // Give the robot some slack time to intercept the ball.
        // The slack time is reduced as we get closer to the ball and have
        // a better chance of intercepting it.
        Duration slack_time_sec =
                std::min(ball_time_to_pos,
                         Duration::fromSeconds(0.1));

        if (robot_time_to_pos < ball_time_to_pos)
        {
            break;
        }
        intercept_position +=
            ball.velocity().normalize(intercept_position_search_interval);
    }
    return intercept_position;
}

Point robotPositionToFaceBall(const Point &ball_position, const Angle &face_ball_angle,
                              double additional_offset)
{
    return ball_position - Vector::createFromAngle(face_ball_angle)
                               .normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                          BALL_MAX_RADIUS_METERS + additional_offset);
}
