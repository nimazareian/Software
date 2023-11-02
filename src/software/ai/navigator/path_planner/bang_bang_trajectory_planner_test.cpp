#include "software/ai/navigator/path_planner/trajectory_planner.h"
#include <gtest/gtest.h>
#include <math.h>
#include <random>

#include "software/test_util/test_util.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "proto/parameters.pb.h"

class BangBangTrajectoryPlannerTest : public testing::Test {
public:       
    BangBangTrajectoryPlannerTest() : planner(), robot_navigation_obstacle_factory(config), rng(1010), pos_uniform_dist(-1, 1), vel_uniform_dist(-1, 1) {}

    TrajectoryPlanner planner;
    RobotNavigationObstacleFactory robot_navigation_obstacle_factory;
    TbotsProto::RobotNavigationObstacleConfig config;
protected:
    static constexpr int num_points = 1000;
    static constexpr double max_x_velocity = 2; 
    static constexpr double max_y_velocity = 2; 
    static constexpr double max_x_position = 3;
    static constexpr double max_y_position = 3;
    static constexpr double maximum_acceleration = 3;

    Vector getRandomVector()
    {
        double x_vel = vel_uniform_dist(rng) * max_x_velocity;
        double y_vel = vel_uniform_dist(rng) * max_y_velocity;

        return Vector(x_vel, y_vel);
    }

    Point getRandomPoint()
    {
        double x_pos = vel_uniform_dist(rng) * max_x_position;
        double y_pos = vel_uniform_dist(rng) * max_y_position;

        return Point(x_pos, y_pos);
    }

    std::mt19937 rng;
    std::uniform_real_distribution<> pos_uniform_dist;
    std::uniform_real_distribution<> vel_uniform_dist;

};

TEST_F(BangBangTrajectoryPlannerTest, generate_path){
    double maximum_velocity = sqrt(max_x_position * max_x_position + max_y_velocity * max_y_velocity);
    KinematicConstraints constraints = {maximum_velocity, BangBangTrajectoryPlannerTest::maximum_acceleration, BangBangTrajectoryPlannerTest::maximum_acceleration};

    for(int i = 0; i<BangBangTrajectoryPlannerTest::num_points; ++i){
        Point start_pos = Point();
        Point destination = getRandomPoint();
        Vector velocity = getRandomVector();

        ObstaclePtr obstacle =
                robot_navigation_obstacle_factory.createFromRobotPosition(getRandomPoint());
        TrajectoryPath path = planner.findTrajectory(start_pos, destination, velocity, constraints, {obstacle}, Field::createSSLDivisionBField().fieldBoundary());
    }
}

TEST_F(BangBangTrajectoryPlannerTest, avoid_obstacle){
    double maximum_velocity = sqrt(max_x_position * max_x_position + max_y_velocity * max_y_velocity);
    KinematicConstraints constraints = {maximum_velocity, BangBangTrajectoryPlannerTest::maximum_acceleration, BangBangTrajectoryPlannerTest::maximum_acceleration};

    Point start_pos = Point(-4.4, -1.05);
    Point destination = Point(-4.4,1.05);
    Vector velocity = Vector(0.0,0);

//    ObstaclePtr obstacle =
//            robot_navigation_obstacle_factory.createFromShape(Circle(Point(0.20,0), 0.18));
    std::vector<ObstaclePtr> obstacles = robot_navigation_obstacle_factory.createStaticObstaclesFromMotionConstraint(TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA, Field::createSSLDivisionBField());
    TrajectoryPath path = planner.findTrajectory(start_pos, destination, velocity, constraints, obstacles, Field::createSSLDivisionBField().fieldBoundary());
}
