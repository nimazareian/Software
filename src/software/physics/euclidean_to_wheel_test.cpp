#include "software/physics/euclidean_to_wheel.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

class EuclideanToWheelTest : public ::testing::Test
{
   protected:
    EuclideanToWheelTest() = default;
    WheelSpace_t current_wheel_speeds{};
    EuclideanSpace_t target_euclidean_velocity{};
    WheelSpace_t expected_wheel_speeds{};

    EuclideanToWheel euclidean_to_four_wheel =
        EuclideanToWheel(create2021RobotConstants());
};

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_zero)
{
    // test +/right
    target_euclidean_velocity = {0, 0, 0};
    expected_wheel_speeds     = {0, 0, 0, 0};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity),
        0.001));
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_x)
{
    // test +/right
    target_euclidean_velocity = {1, 0, 0};
    expected_wheel_speeds     = {-2.2584, -2.2584, 4.0112, 4.0112};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity),
        0.001));

    // test -/left
    target_euclidean_velocity = {-1, 0, 0};
    expected_wheel_speeds     = {2.2584, 2.2584, -4.0112, -4.0112};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity),
        0.001));
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_y)
{
    // test +/forwards
    target_euclidean_velocity = {0, 1, 0};
    expected_wheel_speeds     = {2.1226, -2.1226, -2.7766, 2.7766};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity),
        0.001));

    // test -/backwards
    target_euclidean_velocity = {0, -1, 0};
    expected_wheel_speeds     = {-2.1226, 2.1226, 2.7766, -2.7766};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity),
        0.001));
}

TEST_F(EuclideanToWheelTest, test_double_conversion_y)
{
    // test +/forwards
    double vel_magnitude = 1;
    target_euclidean_velocity = {0, vel_magnitude, 0};
    auto wheel_vel = euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);
    auto euclidean_vel = euclidean_to_four_wheel.getEuclideanVelocity(wheel_vel);
//    euclidean_vel[1] *= -1;
//    euclidean_vel[1] -= vel_magnitude * (2/std::sqrt(2));

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        target_euclidean_velocity,
        euclidean_vel,
        0.001));
}

TEST_F(EuclideanToWheelTest, double_conversion)
{
    // test +/forwards
    std::cout << euclidean_to_four_wheel.wheel_to_euclidean_velocity_D_inverse_ * euclidean_to_four_wheel.euclidean_to_wheel_velocity_D_ << std::endl;
}

TEST_F(EuclideanToWheelTest, test_double_conversion_x)
{
    // test +/forwards
    double vel_magnitude = 1;
    target_euclidean_velocity = {vel_magnitude, 0, 0};
    auto wheel_vel = euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);
    auto euclidean_vel = euclidean_to_four_wheel.getEuclideanVelocity(wheel_vel);
//    euclidean_vel[0] *= -1;
//    euclidean_vel[0] -= vel_magnitude * (2/std::sqrt(2));

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        target_euclidean_velocity,
        euclidean_vel,
        0.001));
}

TEST_F(EuclideanToWheelTest, test_double_conversion_theta)
{
    // test +/forwards
//    double vel_magnitude = 5.3;
    target_euclidean_velocity = {0, 0, 1};
    auto wheel_vel = euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);
    auto euclidean_vel = euclidean_to_four_wheel.getEuclideanVelocity(wheel_vel);
//    euclidean_vel[1] *= -1;
//    euclidean_vel[1] -= vel_magnitude * (2/std::sqrt(2));

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        target_euclidean_velocity,
        euclidean_vel,
        0.001));
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_w)
{
    // test +/forwards
    target_euclidean_velocity = {0, 0, 1};
    expected_wheel_speeds     = {0.0918, 0.0918, 0.0885, 0.0885};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity),
        0.001));

    // test -/backwards
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {0, 0, -1};
    expected_wheel_speeds     = {-0.0918, -0.0918, -0.0885, -0.0885};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity),
        0.001));
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_all)
{
    // test +/forwards
    target_euclidean_velocity = {1, 1, 1};
    expected_wheel_speeds     = {-0.0440, -4.2893, 1.3231, 6.8763};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity),
        0.001));
}

TEST_F(EuclideanToWheelTest, test_sanity_check_conversion_is_linear)
{
    target_euclidean_velocity = {3, 1, 5};

    auto result = euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);

    target_euclidean_velocity = {300, 100, 500};

    auto scaled_result = euclidean_to_four_wheel.getWheelVelocity(
        target_euclidean_velocity);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(result * 100, scaled_result, 0.001));
}
