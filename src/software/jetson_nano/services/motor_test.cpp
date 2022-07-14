#include "software/jetson_nano/services/motor.h"

#include <gtest/gtest.h>
#include "software/test_util/test_util.h"

class MotorServiceTest : public testing::Test
{
   public:
    MotorServiceTest()
    : motor_service(create2021RobotConstants(), 100)
    {
    }

    MotorService motor_service;
};

TEST_F(MotorServiceTest, test_1)
{
    TbotsProto::MotorControl::DirectVelocityControl direct_velocity_control;
    TbotsProto::MotorControl motor_control;

    const Vector zero_vector = Vector();
    *(direct_velocity_control.mutable_velocity()) = *createVectorProto(zero_vector);
    *(direct_velocity_control.mutable_angular_velocity()) = *createAngularVelocityProto(AngularVelocity::zero());

    *(motor_control.mutable_direct_velocity_control()) = direct_velocity_control;

    TbotsProto::MotorStatus motor_status = motor_service.poll(motor_control,
                                         false,
                                         0.01);
}
