#include "software/ai/hl/stp/play/kickoff_enemy_play.h"

#include <gtest/gtest.h>

#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/simulated_tests/validation_functions/robots_on_friendly_half_validation.h"
#include "software/simulated_tests/validation_functions/robots_on_center_circle_validation.h"
#include "software/geom/algorithms/contains.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"
#include "software/world/world.h"

class KickoffEnemyPlayTest : public SimulatedPlayTestFixture
{
};

TEST_F(KickoffEnemyPlayTest, test_kickoff_enemy_play)
{
    setBallState(BallState(Point(0, 0), Vector(0, 0)));
    addFriendlyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(-3, 2.5), Point(-3, 1.5), Point(-3, 0.5), Point(-3, -0.5), Point(-3, -1.5),
         Point(-3, -2.5)}));
    setFriendlyGoalie(0);
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId(
        {Point(1, 0), Point(1, 2.5), Point(1, -2.5), field().enemyGoalCenter(),
         field().enemyDefenseArea().negXNegYCorner(),
         field().enemyDefenseArea().negXPosYCorner()}));
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(KickoffEnemyPlay));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::PREPARE_KICKOFF_THEM);

    std::vector<ValidationFunction> terminating_validation_functions = {
        // TODO: Implement proper validation
        // https://github.com/UBC-Thunderbots/Software/issues/1397
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
            // check that robots 1, 3, 5 are shadowing enemy robots
            auto robotsShadowingEnemy = [](std::shared_ptr<World> world_ptr) -> bool
            {
                // TODO: Fix bug with robot three not shadowing the enemy kicker in kickoff_enemy_play
                // https://github.com/UBC-Thunderbots/Software/issues/

                // Rectangle robotThreeShadowingRect(Point(-0.49, 0.1), Point(-0.75, -0.1));
                Rectangle robotOneShadowingRect(Point(0, 2.2), Point(-0.4, 1.8));
                Rectangle robotFiveShadowingRect(Point(0, -2.2), Point(-0.4, -1.8));

                return contains(robotOneShadowingRect, world_ptr->friendlyTeam().getRobotById(1).value().position()) &&
                       contains(robotFiveShadowingRect, world_ptr->friendlyTeam().getRobotById(5).value().position());
                //     && contains(robotThreeShadowingRect, world_ptr->friendlyTeam().getRobotById(3).value().position());
            };

            // check that robot 2 and 4 are defending the goal posts
            auto robotsDefendingPosts = [](std::shared_ptr<World> world_ptr) -> bool
            {
                // Positions taken from kickoff_enemy_play
                Point robotTwoExpectedPos = Point(world_ptr->field().friendlyGoalpostPos().x() +
                        world_ptr->field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
                        world_ptr->field().defenseAreaYLength() / 2.0);
                Point robotFourExpectedPos = Point(world_ptr->field().friendlyGoalpostNeg().x() +
                        world_ptr->field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
                        -world_ptr->field().defenseAreaYLength() / 2.0);

                // Radius of circle the robots must be within
                double tolerance = 0.05;

                // The expected circles that the robots must be within
                Circle robotTwoCircle(robotTwoExpectedPos, tolerance);
                Circle robotFourCircle(robotFourExpectedPos, tolerance);

                return contains(robotTwoCircle, world_ptr->friendlyTeam().getRobotById(2).value().position()) &&
                       contains(robotFourCircle, world_ptr->friendlyTeam().getRobotById(4).value().position());
            };

            while (!robotsDefendingPosts(world_ptr) && !robotsShadowingEnemy(world_ptr)) {
                yield();
            }
        }};

    std::vector<ValidationFunction> non_terminating_validation_functions = {
        [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) 
        {
            robotsOnFriendlyHalf(world_ptr, yield);
            robotsOnCenterCircle(world_ptr, yield);
        }
    };

    runTest(terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}
