import math
import pytest

import software.python_bindings as tbots
from software.simulated_tests.robot_enters_region import *
# from software.simulated_tests.ball_enters_region import *
# from software.simulated_tests.ball_moves_forward import *
# from software.simulated_tests.friendly_has_ball_possession import *
# from software.simulated_tests.ball_speed_threshold import *
# from software.simulated_tests.robot_speed_threshold import *
# from software.simulated_tests.excessive_dribbling import *
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


@pytest.mark.parametrize(
    "robot_initial_position,robot_initial_orientation,robot_destination,robot_desired_orientation",
    [
        # (
        #     Point(x_meters=0.0, y_meters=0.0),
        #     Angle(radians=0.0),
        #     Point(x_meters=0.0, y_meters=0.0),
        #     Angle(radians=math.pi),
        # ),
        (
                Point(x_meters=-2.5, y_meters=0.0),
                Angle(radians=0.0),
                Point(x_meters=0.0, y_meters=-2.5),
                Angle(radians=math.pi), #math.pi
        ),
        # (
        #     Point(x_meters=0.0, y_meters=0.0),
        #     Angle(radians=0.0),
        #     Point(x_meters=0, y_meters=2.8),
        #     Angle(radians=math.pi),
        # ),
        # (
        #     Point(x_meters=0.0, y_meters=0.0),
        #     Angle(radians=0.0),
        #     Point(x_meters=2.8, y_meters=0),
        #     Angle(radians=math.pi),
        # ),
    ],
)
def test_robot_movement(
    robot_initial_position,
    robot_initial_orientation,
    robot_destination,
    robot_desired_orientation,
    simulated_test_runner,
):
    # Setup Robot
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            [],
            blue_robot_states=[
                RobotState(
                    global_position=robot_initial_position,
                    global_orientation=robot_initial_orientation,
                )
            ],
            ball_location=tbots.Point(4.5, 3.0),
            ball_velocity=tbots.Vector(0.0, 0.0),
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
    params.assigned_tactics[0].move.CopyFrom(
        MoveTactic(
            destination=robot_destination,
            final_orientation=robot_desired_orientation,
            final_speed=0.0,
            dribbler_mode=DribblerMode.OFF,
            ball_collision_type=BallCollisionType.ALLOW,
            auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
            max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
            target_spin_rev_per_s=0.0,
        )
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
    always_validation_sequence_set = [[]]

    # Eventually Validation
    eventually_validation_sequence_set = [[]]

    simulated_test_runner.run_test(
        test_timeout_s=6,
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
    )


if __name__ == "__main__":
    pytest_main(__file__)
