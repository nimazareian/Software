#include "software/geom/algorithms/pid.h"

#include <gtest/gtest.h>
#include "software/logger/logger.h"

TEST(PIDTest, test_constructor)
{
    double time_step = 0.025;
    double max_vel = 4.0;
    double max = max_vel * time_step;
    double min = -max_vel * time_step;
    double Kp = 0.1;
    double Kd = 0.01;
    double Ki = 0.0;
    PID pid = PID(time_step, max, min, Kp, Kd, Ki);

    double val = 20;
    for (int i = 0; i < 500; i++) {
        double inc = pid.calculate(0, val);
        LOG(INFO) << "val: " << val << " + inc: " << inc;
        val += inc;
    }
}
