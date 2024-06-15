from datetime import time

from software.thunderscope.replay.proto_logger import ProtoLogger
from software.thunderscope.proto_unix_io import ProtoUnixIO
from proto.import_all_protos import *
from software.py_constants import *

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


# 1. Create three proto unix ios
# 2. Attach a unix receiver to each

class ProtoLoggerMain:

    def __init__(self):
        self.blue_proto_logger = ProtoUnixIO()
        self.yellow_proto_unix_io = ProtoUnixIO()
        self.logger = ProtoLogger("/tmp/tbots/logs")

        for full_system_proto_unix_io, full_system_runtime_dir in zip([self.blue_proto_logger, self.yellow_proto_unix_io], ["/tmp/tbots/blue", "/tmp/tbots/yellow"]):
            for proto_class in [
                PathVisualization,
                PassVisualization,
                CostVisualization,
                NamedValue,
                PlayInfo,
                ObstacleList,
                DebugShapesMap,
            ]:
                full_system_proto_unix_io.attach_unix_receiver(
                    runtime_dir=full_system_runtime_dir,
                    proto_class=proto_class,
                    from_log_visualize=True,
                )

            full_system_proto_unix_io.attach_unix_receiver(
                full_system_runtime_dir, "/log", RobotLog
            )
    
            # Outputs from full_system
            full_system_proto_unix_io.attach_unix_receiver(
                full_system_runtime_dir, WORLD_PATH, World
            )
            full_system_proto_unix_io.attach_unix_receiver(
                full_system_runtime_dir, PRIMITIVE_PATH, PrimitiveSet
            )

            full_system_proto_unix_io.register_to_observe_everything(self.logger.buffer)


if __name__ == "__main__":
    logger = ProtoLoggerMain()
    while True:
        time.sleep(1)
        pass