"""
bag_recorder.py
"""

import sys
import argparse
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.time import Time, Duration
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import ExtrapolationException
import tf2_geometry_msgs
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata


class BagRecorder(Node):
    """Bag recorder node"""

    def __init__(self, drones: list, use_sim_time: bool, log_file: str, verbose: bool) -> None:
        super().__init__("bag_recorder")

        self.param_use_sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([self.param_use_sim_time])

        self.timer_freq = 1.0

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.start_timestamp: Time

        self.drones = drones
        self.last_centroid_pose = None
        self.last_ref_poses = {drone: None for drone in self.drones}
        self.last_poses = {drone: None for drone in self.drones}
        self.last_twists = {drone: None for drone in self.drones}

        # TODO(pariaspe): depth affect on lookup errors, jump in transformed poses when happen
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST,
                                 depth=10)

        for drone in self.drones:
            self.create_subscription(
                msg_type=PoseStamped,
                topic=f"/{drone}/self_localization/pose",
                callback=lambda msg, drone=drone: self.pose_cbk(msg, drone),
                qos_profile=qos_profile,
            )

            self.create_subscription(
                msg_type=TwistStamped,
                topic=f"/{drone}/self_localization/twist",
                callback=lambda msg, drone=drone: self.twist_cbk(msg, drone),
                qos_profile=qos_profile,
            )

        self._timer: Timer
        self.create_service(Trigger, "/evaluator/start", self.start_cbk)

        # Bagger
        self.bag_topics = set()
        self.bag_writer = SequentialWriter()
        storage_options = StorageOptions(uri=f"rosbags/{log_file}", storage_id="sqlite3")
        converter_options = ConverterOptions(
            input_serialization_format="", output_serialization_format="")
        self.bag_writer.open(storage_options, converter_options)

        self.get_logger().info("Bag recorder ready")

    def pose_cbk(self, msg: PoseStamped, drone_id: str) -> None:
        """Callback for pose"""
        self.last_poses[drone_id] = msg

        self.get_logger().info(f"Received pose from {drone_id}")
        # Centroid
        # TODO(pariaspe): done for every drone, should be done once
        centroid = PoseStamped()
        centroid.header = msg.header
        centroid.header.frame_id = "Swarm/Swarm"
        try:
            centroid_in_earth = self.buffer.transform(
                centroid, "earth", timeout=Duration(seconds=1))
            self.last_centroid_pose = centroid_in_earth
            self.get_logger().info(f"Transformation done: {centroid_in_earth}")
        except ExtrapolationException as e:
            self.get_logger().error(f"Extrapolation error: {e}")

        # Drone reference poses
        ref_pose = PoseStamped()
        ref_pose.header = msg.header
        ref_pose.header.frame_id = f"Swarm/{drone_id}_ref"
        ref_pose_in_earth = self.buffer.transform(ref_pose, "earth")
        self.last_ref_poses[drone_id] = ref_pose_in_earth

    def twist_cbk(self, msg: TwistStamped, drone_id: str) -> None:
        """Callback for twist"""
        self.last_twists[drone_id] = msg

        # TODO(pariaspe): Implement centroid twist in earth and drone twist in Swarm/Swarm frame

    def start_cbk(self, request: Trigger.Request, response: Trigger.Response) -> None:
        """Start the evaluation"""
        _ = (request,)
        self._timer = self.create_timer(self.timer_freq, self.evaluate)

        # Register topics in rosbag
        for drone_id in self.drones:
            self.register_topic_to_bag(
                f"/metrics/{drone_id}/pose", "geometry_msgs/msg/PoseStamped")
            self.register_topic_to_bag(
                f"/metrics/{drone_id}/ref_pose", "geometry_msgs/msg/PoseStamped")
            self.register_topic_to_bag(
                f"/metrics/{drone_id}/twist", "geometry_msgs/msg/TwistStamped")
        self.register_topic_to_bag("/metrics/centroid/pose", "geometry_msgs/msg/PoseStamped")

        self.evaluate()

        response.success = True
        return response

    def evaluate(self) -> None:
        """Write to rosbag"""
        # TODO(pariaspe): use the same timestamp for all drones or different?
        t = self.get_clock().now()
        for drone_id in self.drones:
            self.bag_writer.write(f"/metrics/{drone_id}/pose",
                                  serialize_message(self.last_poses[drone_id]), t)
            self.bag_writer.write(f"/metrics/{drone_id}/ref_pose",
                                  serialize_message(self.last_ref_poses[drone_id]), t)
            self.bag_writer.write(f"/metrics/{drone_id}/twist",
                                  serialize_message(self.last_twists[drone_id]), t)

        self.bag_writer.write("/metrics/centroid/pose",
                              serialize_message(self.last_centroid_pose), t)
        self.get_logger().info(str(t.seconds_nanoseconds()), throttle_duration_sec=30.0)

    def register_topic_to_bag(self, topic: str, msg_type: str) -> None:
        """Register topic to bag"""
        if topic in self.bag_topics:
            return
        topic_info = TopicMetadata(
            name=topic,
            type=msg_type,
            serialization_format="cdr"
        )
        self.bag_writer.create_topic(topic_info)


if __name__ == "__main__":
    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file_default = f"rosbag2_{now}"

    parser = argparse.ArgumentParser(
        prog="bag_recorder.py", description="Bag recorder script", add_help=True)
    parser.add_argument("-s", "--use-sim-time", action="store_true",
                        help="Use simulation time")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Verbose mode")
    parser.add_argument("-l", "--log-file", type=str,
                        default=log_file_default, help="Log file")
    parser.add_argument("drones", type=str,
                        default=log_file_default, help="Drone list, separated by comma")
    args = parser.parse_args()

    rclpy.init()

    print("Saving log at", args.log_file)
    recorder = BagRecorder(args.drones.split(","),
                           args.use_sim_time, args.log_file, args.verbose)

    rclpy.spin(recorder)

    rclpy.shutdown()
    sys.exit(0)
