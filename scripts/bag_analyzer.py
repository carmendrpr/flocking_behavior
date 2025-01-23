"""
bag_analyzer.py
"""

from dataclasses import dataclass, field
from math import sqrt
from pathlib import Path
import matplotlib.pyplot as plt

from rclpy.time import Duration
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
import tf2_geometry_msgs

from bag_reader import read_rosbag, deserialize_msgs, deserialize_tfs


def timestamp_to_float(header: Header) -> float:
    """Parse timestamp from header and convert float"""
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def derivate_pose(ps_history: list[PoseStamped], tws_history: list[TwistStamped],
                  alpha: float, epsilon: float = 0.00001) -> TwistStamped:
    """Calculate twist from pose discrete derivation"""
    ps0 = ps_history[-2] if len(ps_history) > 1 else None
    ps1 = ps_history[-1] if len(ps_history) > 0 else None
    tws0 = tws_history[-1] if len(tws_history) > 0 else None
    if not ps0 or not ps1:
        tw = TwistStamped()
        tw.header.stamp = ps1.header.stamp
        tw.header.frame_id = 'Swarm/Swarm'
        return tw
    dt = timestamp_to_float(ps1.header) - timestamp_to_float(ps0.header)
    dt = dt if dt > epsilon else epsilon
    dx = ps1.pose.position.x - ps0.pose.position.x
    dy = ps1.pose.position.y - ps0.pose.position.y
    dz = ps1.pose.position.z - ps0.pose.position.z
    tws1 = TwistStamped()
    tws1.header.stamp = ps1.header.stamp
    tws1.header.frame_id = tws0.header.frame_id
    # smooth filtering
    tws1.twist.linear.x = alpha * dx / dt + (1.0 - alpha) * tws0.twist.linear.x
    tws1.twist.linear.y = alpha * dy / dt + (1.0 - alpha) * tws0.twist.linear.y
    tws1.twist.linear.z = alpha * dz / dt + (1.0 - alpha) * tws0.twist.linear.z
    return tws1


@dataclass
class LogData:
    """Data read from rosbag file"""
    filename: Path
    poses: dict[str, list[PoseStamped]] = field(default_factory=dict)
    twists: dict[str, list[TwistStamped]] = field(default_factory=dict)
    centroid_poses: list[PoseStamped] = field(default_factory=list)
    centroid_twists: list[TwistStamped] = field(default_factory=list)
    ref_poses: dict[str, list[PoseStamped]] = field(default_factory=dict)
    twists_in_swarm: dict[str, list[TwistStamped]] = field(default_factory=dict)
    traj: list[PoseStamped] = field(default_factory=list)

    @ classmethod
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
                poses = deserialize_msgs(msgs, PoseStamped)
                drone_id = topic.split("/")[1]
                log_data.poses[drone_id] = []
                log_data.ref_poses[drone_id] = []
                log_data.twists_in_swarm[drone_id] = []
                poses_in_swarm = []  # aux

                for pose in poses:
                    log_data.poses[drone_id].append(pose)

                    centroid = PoseStamped()
                    centroid.header.stamp = pose.header.stamp
                    centroid.header.frame_id = 'Swarm/Swarm'
                    if buffer.can_transform('earth', 'Swarm/Swarm', pose.header.stamp):
                        centroid_in_earth = buffer.transform(centroid, 'earth')
                        if drone_id == "drone0":
                            log_data.centroid_poses.append(centroid_in_earth)
                            log_data.centroid_twists.append(derivate_pose(
                                log_data.centroid_poses, log_data.centroid_twists, 0.01))

                        # do static transform manually
                        ref_pose = tf2_geometry_msgs.do_transform_pose_stamped(
                            centroid_in_earth, tf_static[f'Swarm/Swarm_Swarm/{drone_id}_ref'])
                        log_data.ref_poses[drone_id].append(ref_pose)

                    if buffer.can_transform('Swarm/Swarm', pose.header.frame_id, pose.header.stamp):
                        pose_in_swarm = buffer.transform(pose, 'Swarm/Swarm')
                        poses_in_swarm.append(pose_in_swarm)
                        log_data.twists_in_swarm[drone_id].append(derivate_pose(
                            poses_in_swarm, log_data.twists_in_swarm[drone_id], 0.01))
            elif "self_localization/twist" in topic:
                drone_id = topic.split("/")[1]
                log_data.twists[drone_id] = deserialize_msgs(msgs, TwistStamped)
            elif "/tf" == topic:
                continue
            elif '/tf_static' == topic:
                continue
            elif '/Swarm/debug/traj_generated' == topic:
                log_data.traj = deserialize_msgs(msgs, PoseStamped)
            else:
                print(f"Ignored topic: {topic}")
                continue
            print(f'Processed {topic}')

        return log_data

    def __str__(self):
        """Print stats"""
        text = f"{self.filename.stem}\n"
        return text


def plot_path(data: LogData):
    """Plot paths"""
    fig, ax = plt.subplots()
    for drone, poses in zip(data.poses.keys(), data.poses.values()):
        # https://stackoverflow.com/questions/52773215
        x = [pose.pose.position.x for pose in poses]
        y = [pose.pose.position.y for pose in poses]
        ax.plot(x, y, label=drone)

    # for drone, ref_poses in zip(data.ref_poses.keys(), data.ref_poses.values()):
    #     x = [pose.pose.position.x for pose in ref_poses]
    #     y = [pose.pose.position.y for pose in ref_poses]
    #     ax.plot(x, y, label=f'{drone}_ref')

    x = [pose.pose.position.x for pose in data.centroid_poses]
    y = [pose.pose.position.y for pose in data.centroid_poses]
    ax.plot(x, y, label='centroid')

    ax.set_title(f'Path {data.filename.stem}')
    ax.set_xlabel('y (m)')
    ax.set_ylabel('x (m)')
    ax.legend()
    ax.grid()
    fig.savefig(f"/tmp/path_{data.filename.stem}.png")
    return fig


def plot_x(data: LogData):
    fig, ax = plt.subplots()
    for drone, poses in zip(data.poses.keys(), data.poses.values()):
        x = [pose.pose.position.x for pose in poses]
        ts = [timestamp_to_float(pose.header) - timestamp_to_float(poses[0].header)
              for pose in poses]
        ax.plot(ts, x, label=drone)

    ax.set_title(f'X {data.filename.stem}')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('x (m)')
    ax.legend()
    ax.grid()
    fig.savefig(f"/tmp/x_{data.filename.stem}.png")
    return fig


def plot_twist(data: LogData):
    """Plot twists"""
    fig, ax = plt.subplots()
    for drone, twists in zip(data.twists.keys(), data.twists.values()):
        sp = [sqrt(twist.twist.linear.x**2 + twist.twist.linear.y ** 2 + twist.twist.angular.z**2)
              for twist in twists]
        ts = [timestamp_to_float(twist.header) - timestamp_to_float(twists[0].header)
              for twist in twists]
        ax.plot(ts, sp, label=drone)

    sp = [sqrt(twist.twist.linear.x**2 + twist.twist.linear.y ** 2 + twist.twist.angular.z**2)
          for twist in data.centroid_twists]
    ts = [timestamp_to_float(twist.header) - timestamp_to_float(data.centroid_twists[0].header)
          for twist in data.centroid_twists]
    ax.plot(ts, sp, label='centroid')

    ax.set_title(f'Twists {data.filename.stem}')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('twist (m/s)')
    ax.legend()
    ax.grid()
    fig.savefig(f"/tmp/twist_{data.filename.stem}.png")
    return fig


def plot_twist_in_swarm(data: LogData):
    """Plot twists"""
    fig, ax = plt.subplots()
    for drone, twists in zip(data.twists_in_swarm.keys(), data.twists_in_swarm.values()):
        sp = [sqrt(twist.twist.linear.x**2 + twist.twist.linear.y ** 2 + twist.twist.angular.z**2)
              for twist in twists]
        ts = [timestamp_to_float(twist.header) - timestamp_to_float(twists[0].header)
              for twist in twists]
        ax.plot(ts, sp, label=drone)

    sp = [sqrt(twist.twist.linear.x**2 + twist.twist.linear.y ** 2 + twist.twist.angular.z**2)
          for twist in data.centroid_twists]
    ts = [timestamp_to_float(twist.header) - timestamp_to_float(data.centroid_twists[0].header)
          for twist in data.centroid_twists]
    ax.plot(ts, sp, label='centroid')

    ax.set_title(f'Twists {data.filename.stem}')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('twist (m/s)')
    ax.legend()
    ax.grid()
    fig.savefig(f"/tmp/twist_in_swarm_{data.filename.stem}.png")
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
        plot_x(data)
        plot_twist_in_swarm(data)
        # print(data.stats(25.0))
        plt.show()


if __name__ == "__main__":
    # main('rosbags/test2')
    # main('rosbags/Experimentos/Lineal_Vel_1/')
    main('rosbags/Experimentos/Curva_Vel_05/rosbag2_2025_01_23-12_26_16/')
    # main('rosbags/Experimentos/Curva_Vel_05/rosbag2_2025_01_23-12_23_57/')
