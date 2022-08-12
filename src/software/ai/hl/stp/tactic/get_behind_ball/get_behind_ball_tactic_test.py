import pytest

import software.python_bindings as tbots
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_moves_forward import *
from software.simulated_tests.friendly_has_ball_possession import *
from software.simulated_tests.ball_speed_threshold import *
from software.simulated_tests.robot_speed_threshold import *
from software.simulated_tests.excessive_dribbling import *
from proto.geometry_pb2 import Point, Angle
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


@pytest.mark.parametrize(
    "ball_initial_position,ball_initial_velocity,robot_initial_position",
    [
        # ball moving down and out goal from defense area
        (
            tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
            + tbots.Vector(0.3, 0),
            tbots.Vector(0, -0.7),
            tbots.Point(-3.5, 0),
        ),
    ],
)
def test_robot_moves_behind_ball(
    ball_initial_position,
    ball_initial_velocity,
    robot_initial_position,
    simulated_test_runner,
):
    # Setup Robot
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            [],
            blue_robot_locations=[robot_initial_position],
            ball_location=ball_initial_position,
            ball_velocity=ball_initial_velocity,
        ),
    )

    # These aren't necessary for this test, but this is just an example
    # of how to send commands to the simulator.
    #
    # NOTE: The gamecontroller responses are automatically handled by
    # the gamecontroller context manager class
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.FORCE_START, team=Team.BLUE
    )

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].get_behind_ball.CopyFrom(
        GetBehindBallTactic(ball_location=Point(x_meters=0, y_meters=0),
                            chick_direction=Angle(radians=tbots.Angle.fromDegrees(180).toRadians()))
    )
    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Always Validation
    always_validation_sequence_set = []

    # Eventually Validation
    eventually_validation_sequence_set = []

    simulated_test_runner.run_test(
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
    )


if __name__ == "__main__":
    pytest_main(__file__)
