#include "software/geom/algorithms/pid.h"

#include <gtest/gtest.h>
#include "software/logger/logger.h"

TEST(PIDTest, test_constructor)
{
    PID pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5);

    double val = 20;
    for (int i = 0; i < 100; i++) {
        double inc = pid.calculate(0, val);
        LOG(INFO) << "val: " << val << " + inc: " << inc;
        val += inc;
    }
}
