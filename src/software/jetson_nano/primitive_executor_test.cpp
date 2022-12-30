#include "software/geom/angle.h"
#include "software/geom/vector.h"

#include <gtest/gtest.h>

TEST(AngleTest, Statics)
{
    Angle orientation = Angle::fromDegrees(90);
    Vector global(1, 0);
    Vector local = global.rotate(-orientation);
    EXPECT_DOUBLE_EQ(local.x(), 0);
    EXPECT_DOUBLE_EQ(local.y(), -1);
}

TEST(AngleTest, Statics2)
{
    Angle orientation = Angle::fromDegrees(135);
    Vector global(1, 0);
    Vector local = global.rotate(-orientation);
    EXPECT_DOUBLE_EQ(local.x(), 0);
    EXPECT_DOUBLE_EQ(local.y(), -1);
}
