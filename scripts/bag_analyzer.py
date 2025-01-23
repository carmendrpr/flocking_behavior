"""
bag_analyzer.py
"""

from dataclasses import dataclass, field
from math import sqrt
from pathlib import Path
import matplotlib.pyplot as plt

from rclpy.time import Duration
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped, TwistStamped
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
import tf2_geometry_msgs

from bag_reader import read_rosbag, deserialize_msgs, deserialize_tfs


@dataclass
class LogData:
    """Data read from rosbag file"""
    filename: Path
    timestamps: list[float] = field(default_factory=list)
    poses: dict[str, list[tuple[float, float]]] = field(default_factory=dict)
    twists: dict[str, list[tuple[float, float, float, float]]] = field(default_factory=dict)
    centroid_poses: list[tuple[float, float]] = field(default_factory=list)
    ref_poses: dict[str, list[tuple[float, float]]] = field(default_factory=dict)

    @classmethod
    def from_rosbag(cls, rosbag: Path) -> 'LogData':
        """Read the rosbag"""
        log_data = cls(rosbag)
        rosbag_msgs = read_rosbag(str(rosbag))

        buffer = Buffer(cache_time=Duration(seconds=1200))
        print('Processing /tf_static...')
        buffer = deserialize_tfs(rosbag_msgs['/tf_static'], buffer)
        print('Processing /tf...')
        buffer = deserialize_tfs(rosbag_msgs['/tf'], buffer)

        tf_static: dict[str, TransformStamped] = {}
        tfs = deserialize_msgs(rosbag_msgs['/tf_static'], TFMessage)
        for tf in tfs:
            for transform in tf.transforms:
                k = f'{transform.header.frame_id}_{transform.child_frame_id}'
                tf_static[k] = transform

        for topic, msgs in rosbag_msgs.items():
            if "self_localization/pose" in topic:
                poses = deserialize_msgs(msgs, PointStamped)
                drone_id = topic.split("/")[1]
                log_data.poses[drone_id] = []
                log_data.ref_poses[drone_id] = []
                ts0 = log_data.parse_timestamp(poses[0].header)

                for pose in poses:
                    log_data.poses[drone_id].append((pose.point.x, pose.point.y))

                    centroid = PoseStamped()
                    centroid.header = pose.header
                    centroid.header.frame_id = 'Swarm/Swarm'
                    if buffer.can_transform('earth', 'Swarm/Swarm', pose.header.stamp):
                        centroid_in_earth = buffer.transform(centroid, 'earth')
                        if drone_id == "drone0":
                            log_data.timestamps.append(log_data.parse_timestamp(pose.header) - ts0)
                            log_data.centroid_poses.append((centroid_in_earth.pose.position.x,
                                                            centroid_in_earth.pose.position.y))

                        # do static transform manually
                        ref_pose = tf2_geometry_msgs.do_transform_pose(
                            centroid_in_earth.pose, tf_static[f'Swarm/Swarm_Swarm/{drone_id}_ref'])
                        log_data.ref_poses[drone_id].append((ref_pose.position.x,
                                                            ref_pose.position.y))
            elif "self_localization/twist" in topic:
                twists = deserialize_msgs(msgs, TwistStamped)
                drone_id = topic.split("/")[1]
                log_data.twists[drone_id] = []
                for twist in twists:
                    speed = sqrt(twist.twist.linear.x**2 + twist.twist.linear.y **
                                 2 + twist.twist.angular.z**2)
                    log_data.twists[drone_id].append(
                        (speed, twist.twist.linear.x, twist.twist.linear.y, twist.twist.angular.z))
            elif "/tf" == topic:
                continue
            elif '/tf_static' == topic:
                continue
            else:
                print(f"Unknown topic: {topic}")
                continue
            print(f'Processed {topic}')

        return log_data

    def parse_timestamp(self, header: Header) -> float:
        """Parse timestamp from header"""
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def __str__(self):
        """Print stats"""
        text = f"{self.filename.stem}\n"
        text += f"From {self.timestamps[0]:8.2f}s "
        text += f"to {self.timestamps[-1]:8.2f}s "
        text += f'{self.poses["drone0"][0]} '
        text += f'{self.poses["drone0"][-1]}\n'
        return text


def plot_path(data: LogData):
    """Plot paths"""
    fig, ax = plt.subplots()
    for drone, poses in zip(data.poses.keys(), data.poses.values()):
        x, y = zip(*poses)
        ax.plot(x, y, label=drone)

    for drone, ref_poses in zip(data.ref_poses.keys(), data.ref_poses.values()):
        x, y = zip(*ref_poses)
        ax.plot(x, y, label=f'{drone}_ref')

    x, y = zip(*data.centroid_poses)
    ax.plot(x, y, label='centroid')

    ax.set_title(f'Path {data.filename.stem}')
    ax.set_xlabel('y (m)')
    ax.set_ylabel('x (m)')
    ax.legend()
    ax.grid()
    fig.savefig(f"/tmp/path_{data.filename.stem}.png")
    return fig


def plot_twist(data: LogData):
    """Plot twists"""
    fig, ax = plt.subplots()
    for drone, twists in zip(data.twists.keys(), data.twists.values()):
        sp, x, y, z = zip(*twists)
        max_len = min(len(data.timestamps), len(sp))
        ax.plot(data.timestamps[:max_len], sp[:max_len], label=drone)

    ax.set_title(f'Twists {data.filename.stem}')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('twist (m/s)')
    ax.legend()
    ax.grid()
    fig.savefig(f"/tmp/twist_{data.filename.stem}.png")
    return fig


def main(log_file: str):
    """Main function"""
    if Path(log_file).is_dir():
        log_files = list(Path(log_file).iterdir())
        for child in Path(log_file).iterdir():
            if child.is_file() and child.suffix == ".db3":
                log_files = [Path(log_file)]
                break
    elif Path(log_file).is_file():
        raise NotADirectoryError(f"{log_file} is not a directory")

    fig, fig2 = None, None
    for log in log_files:
        data = LogData.from_rosbag(log)

        print(data)
        # fig = plot_area(data, fig)
        # fig2 = plot_total_path(data, fig2)

        fig = plot_path(data)
        fig2 = plot_twist(data)
        # print(data.stats(25.0))
        plt.show()


if __name__ == "__main__":
    # main('rosbags/test2')
    main('rosbags/Experimentos/Lineal_Vel_1/')
