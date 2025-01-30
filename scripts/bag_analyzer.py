"""
bag_analyzer.py
"""

from collections import deque
import copy
from dataclasses import dataclass, field
from math import sqrt
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

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


def distance(pose1: PoseStamped, pose2: PoseStamped, plane: bool = False) -> float:
    """Calculate distance between two poses"""
    dx = pose1.pose.position.x - pose2.pose.position.x
    dy = pose1.pose.position.y - pose2.pose.position.y
    dz = pose1.pose.position.z - pose2.pose.position.z
    if plane:
        return sqrt(dx**2 + dy**2)
    return sqrt(dx**2 + dy**2 + dz**2)


def twist_to_polar_vector(twist: TwistStamped) -> tuple[float, float]:
    """Convert twist to polar 3d vector"""
    x = twist.twist.linear.x
    y = twist.twist.linear.y
    z = twist.twist.linear.z
    r = sqrt(x**2 + y**2 + z**2)
    theta = np.arctan2(y, x)
    try:
        phi = np.arccos(z / r)
    except ZeroDivisionError:
        phi = 0.0
    return r, theta, phi


@dataclass
class LogData:
    """Data read from rosbag file"""
    filename: Path
    poses: dict[str, list[PoseStamped]] = field(default_factory=dict)
    twists: dict[str, list[TwistStamped]] = field(default_factory=dict)
    centroid_poses: list[PoseStamped] = field(default_factory=list)
    centroid_twists: list[TwistStamped] = field(default_factory=list)
    ref_poses: dict[str, list[PoseStamped]] = field(default_factory=dict)
    poses_in_swarm: dict[str, list[PoseStamped]] = field(default_factory=dict)
    twists_in_swarm: dict[str, list[TwistStamped]] = field(default_factory=dict)
    traj: list[PoseStamped] = field(default_factory=list)
    tf_static: dict[str, TransformStamped] = field(default_factory=dict)

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

        tfs = deserialize_msgs(rosbag_msgs['/tf_static'], TFMessage)
        for tf in tfs:
            for transform in tf.transforms:
                k = f'{transform.header.frame_id}_{transform.child_frame_id}'
                log_data.tf_static[k] = transform

        for topic, msgs in rosbag_msgs.items():
            if "self_localization/pose" in topic:
                poses = deserialize_msgs(msgs, PoseStamped)
                drone_id = topic.split("/")[1]
                log_data.poses[drone_id] = []
                log_data.ref_poses[drone_id] = []
                log_data.twists_in_swarm[drone_id] = []
                log_data.poses_in_swarm[drone_id] = []

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
                            centroid_in_earth, log_data.tf_static[f'Swarm/Swarm_Swarm/{drone_id}_ref'])
                        ref_pose.header.frame_id = 'earth'
                        log_data.ref_poses[drone_id].append(copy.deepcopy(ref_pose))

                    if buffer.can_transform('Swarm/Swarm', pose.header.frame_id, pose.header.stamp):
                        pose_in_swarm = buffer.transform(pose, 'Swarm/Swarm')
                        log_data.poses_in_swarm[drone_id].append(pose_in_swarm)
                        log_data.twists_in_swarm[drone_id].append(derivate_pose(
                            log_data.poses_in_swarm[drone_id], log_data.twists_in_swarm[drone_id], 0.01))
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

    def cohesion_metric(self, t0: float = None) -> dict[str, tuple[float, float]]:
        drone_to_centroid_distances = {}
        for drone, poses in zip(self.poses_in_swarm.keys(), self.poses_in_swarm.values()):
            for pose in poses:
                if t0 and timestamp_to_float(pose.header) < t0:
                    continue
                if drone not in drone_to_centroid_distances:
                    drone_to_centroid_distances[drone] = []
                drone_to_centroid_distances[drone].append(distance(pose, PoseStamped()))

        drone_to_centroid_mean_distances = {}
        for k, distances in drone_to_centroid_distances.items():
            drone_to_centroid_mean_distances[k] = (np.mean(distances), np.std(distances))

        return drone_to_centroid_mean_distances

    def separation_metric(self, t0: float = None) -> dict[str, tuple[float, float]]:
        drone_to_drone_distances = {}
        for drone, poses in zip(self.poses.keys(), self.poses.values()):
            for other_drone, other_poses in zip(self.poses.keys(), self.poses.values()):
                if drone == other_drone:
                    continue
                for pose, other_pose in zip(poses, other_poses):
                    if t0 and timestamp_to_float(pose.header) < t0:
                        continue
                    if f'{drone}_{other_drone}' not in drone_to_drone_distances:
                        drone_to_drone_distances[f'{drone}_{other_drone}'] = []
                    drone_to_drone_distances[f'{drone}_{other_drone}'].append(
                        distance(pose, other_pose))

        drone_to_drone_mean_distances = {}
        for k, distances in drone_to_drone_distances.items():
            drone_to_drone_mean_distances[k] = (np.mean(distances), np.std(distances))

        return drone_to_drone_mean_distances

    def alignment_metric(self, t0: float = None) -> dict[str, tuple[float, float]]:
        drone_to_centroid_twists = {}
        for drone, twists in zip(self.twists_in_swarm.keys(), self.twists_in_swarm.values()):
            for twist in twists:
                if t0 and timestamp_to_float(twist.header) < t0:
                    continue
                if drone not in drone_to_centroid_twists:
                    drone_to_centroid_twists[drone] = []
                r, theta, phi = twist_to_polar_vector(twist)
                drone_to_centroid_twists[drone].append((r, theta, phi))

        drone_to_centroid_mean_twists = {}
        for k, twists in drone_to_centroid_twists.items():
            r, theta, phi = zip(*twists)
            drone_to_centroid_mean_twists[k] = (np.mean(r), np.std(r))
            # drone_to_centroid_mean_twists[k] = (np.mean(r), np.std(r), np.mean(theta), np.std(theta),
            #                                     np.mean(phi), np.std(phi))
        return drone_to_centroid_mean_twists

    def ref_error_metric(self, t0: float = None) -> dict[str, tuple[float, float]]:
        drone_to_ref_distances = {}
        for drone, poses in zip(self.poses_in_swarm.keys(), self.poses_in_swarm.values()):
            ref = PoseStamped()
            ref.header.frame_id = f'Swarm/{drone}_ref'
            ref.pose.position.x = self.tf_static[f'Swarm/Swarm_Swarm/{drone}_ref'].transform.translation.x
            ref.pose.position.y = self.tf_static[f'Swarm/Swarm_Swarm/{drone}_ref'].transform.translation.y
            ref.pose.position.z = self.tf_static[f'Swarm/Swarm_Swarm/{drone}_ref'].transform.translation.z

            for pose in poses:
                if t0 and timestamp_to_float(pose.header) < t0:
                    continue
                if drone not in drone_to_ref_distances:
                    drone_to_ref_distances[drone] = []
                drone_to_ref_distances[drone].append(distance(pose, ref, True))

        drone_to_ref_mean_distances = {}
        for k, distances in drone_to_ref_distances.items():
            drone_to_ref_mean_distances[k] = (np.mean(distances), np.std(distances))

        return drone_to_ref_mean_distances


def get_metrics(data: LogData):
    print('------- COHESION -------')
    for k, v in data.cohesion_metric(timestamp_to_float(data.traj[0].header)).items():
        print(f'\t{k}: {v[0]:.3f} ± {v[1]:.3f} [m]')

    print('------- SEPARATION -------')
    for k, v in data.separation_metric(timestamp_to_float(data.traj[0].header)).items():
        print(f'\t{k}: {v[0]:.3f} ± {v[1]:.3f} [m]')

    print('------- ALIGNMENT -------')
    for k, v in data.alignment_metric(timestamp_to_float(data.traj[0].header)).items():
        # print(f'\t{k}: {v[0]:.3f} ± {v[1]:.3f} [m/s]' +
        #       f' | {v[2]:.3f} ± {v[3]:.3f} [rad] | {v[4]:.3f} ± {v[5]:.3f} [rad]')
        print(f'\t{k}: {v[0]:.3f} ± {v[1]:.3f} [m/s]')

    print('------- REFERENCE ERROR -------')
    for k, v in data.ref_error_metric(timestamp_to_float(data.traj[0].header)).items():
        print(f'\t{k}: {v[0]:.3f} ± {v[1]:.3f} [m]')


def plot_path(data: LogData):
    """Plot paths"""
    fig, ax = plt.subplots()
    for drone, poses in zip(data.poses.keys(), data.poses.values()):
        # https://stackoverflow.com/questions/52773215
        x = [pose.pose.position.x for pose in poses]
        y = [pose.pose.position.y for pose in poses]
        ax.plot(x, y, label=drone)

    for drone, ref_poses in zip(data.ref_poses.keys(), data.ref_poses.values()):
        x = [pose.pose.position.x for pose in ref_poses]
        y = [pose.pose.position.y for pose in ref_poses]
        ax.plot(x, y, linestyle='dashed', label=f'{drone}_ref')

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


def plot_colored_path(data: LogData):
    """Plot paths"""
    fig, ax = plt.subplots()
    for drone, poses, twists in zip(data.poses.keys(), data.poses.values(), data.twists.values()):
        # https://stackoverflow.com/questions/52773215
        x = [pose.pose.position.x for pose in poses]
        y = [pose.pose.position.y for pose in poses]
        c = [sqrt(twist.twist.linear.x**2 + twist.twist.linear.y ** 2 + twist.twist.angular.z**2)
             for twist in twists]
        x = x[:len(c)]
        y = y[:len(c)]
        c = c[:len(x)]
        # FIXME(pariaspe): hardcoded
        x = x[1940:]
        y = y[1940:]
        c = c[1940:]

        ax.scatter(x, y, s=1, c=c, cmap='plasma')
    fig.colorbar(ax.collections[0], ax=ax, label='speed (m/s)')

    for drone, poses in zip(data.poses.keys(), data.poses.values()):
        ax.plot(poses[1940].pose.position.x, poses[1940].pose.position.y, 'kD')
        ax.text(poses[1940].pose.position.x - 0.4, poses[1940].pose.position.y + 0.35, drone)

    # x = [pose.pose.position.x for pose in data.centroid_poses]
    # y = [pose.pose.position.y for pose in data.centroid_poses]
    # ax.plot(x, y, label='centroid')

    ax.set_title(f'Path {data.filename.stem}')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    # ax.legend()
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
    t0 = timestamp_to_float(data.traj[0].header)
    fig, ax = plt.subplots()
    for drone, twists in zip(data.twists.keys(), data.twists.values()):
        sp, ts = [], []
        for twist in twists:
            if timestamp_to_float(twist.header) < t0:
                continue
            sp.append(sqrt(twist.twist.linear.x**2 +
                      twist.twist.linear.y ** 2 + twist.twist.angular.z**2))
            ts.append(timestamp_to_float(twist.header) - t0)
        ax.plot(ts, sp, label=drone)

    sp, ts = [], []
    window = deque(maxlen=10)
    for twist in data.centroid_twists:
        if timestamp_to_float(twist.header) < t0:
            continue
        window.append(sqrt(twist.twist.linear.x**2 +
                      twist.twist.linear.y ** 2 + twist.twist.angular.z**2))
        sp.append(np.mean(window))
        ts.append(timestamp_to_float(twist.header) - t0)
    ax.plot(ts, sp, label='centroid')

    ax.set_title(f'Twists {data.filename.stem}')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('twist (m/s)')
    ax.set_ylim(0, 1)
    ax.legend()
    ax.grid()
    fig.savefig(f"/tmp/twist_{data.filename.stem}.png")
    return fig


def plot_twist_in_swarm(data: LogData):
    """Plot twists"""
    t0 = timestamp_to_float(data.traj[0].header)

    fig, ax = plt.subplots()
    for drone, twists in zip(data.twists_in_swarm.keys(), data.twists_in_swarm.values()):
        sp, ts = [], []
        window = deque(maxlen=10)
        for twist in twists:
            if timestamp_to_float(twist.header) < t0:
                continue
            window.append(sqrt(twist.twist.linear.x**2 +
                               twist.twist.linear.y ** 2 + twist.twist.angular.z**2))
            sp.append(np.mean(window))
            ts.append(timestamp_to_float(twist.header) - t0)
        ax.plot(ts, sp, label=drone)

    sp, ts = [], []
    window = deque(maxlen=10)
    for twist in data.centroid_twists:
        if timestamp_to_float(twist.header) < t0:
            continue
        window.append(sqrt(twist.twist.linear.x**2 +
                           twist.twist.linear.y ** 2 + twist.twist.angular.z**2))
        sp.append(np.mean(window))
        ts.append(timestamp_to_float(twist.header) - t0)
    ax.plot(ts, sp, label='centroid')

    ax.set_title(f'Twists in swarm {data.filename.stem}')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('twist (m/s)')
    ax.set_ylim(0, 1)
    ax.legend()
    ax.grid()
    fig.savefig(f"/tmp/twist_in_swarm_{data.filename.stem}.png")
    return fig


def plot_all_twist(data: LogData):
    """Plot twists"""
    t0 = timestamp_to_float(data.traj[0].header)
    c = {'drone0': 'r', 'drone1': 'g', 'drone2': 'b'}

    fig, ax = plt.subplots()
    for drone, twists in zip(data.twists.keys(), data.twists.values()):
        sp, ts = [], []
        for twist in twists:
            if timestamp_to_float(twist.header) < t0:
                continue
            sp.append(sqrt(twist.twist.linear.x**2 +
                      twist.twist.linear.y ** 2 + twist.twist.angular.z**2))
            ts.append(timestamp_to_float(twist.header) - t0)
        ax.plot(ts, sp, c[drone], label=drone)

    for drone, twists in zip(data.twists_in_swarm.keys(), data.twists_in_swarm.values()):
        sp, ts = [], []
        window = deque(maxlen=10)
        for twist in twists:
            if timestamp_to_float(twist.header) < t0:
                continue
            window.append(sqrt(twist.twist.linear.x**2 +
                               twist.twist.linear.y ** 2 + twist.twist.angular.z**2))
            sp.append(np.mean(window))
            ts.append(timestamp_to_float(twist.header) - t0)
        ax.plot(ts, sp, c[drone], linestyle='dotted', label=f'{drone}_rel')

    sp, ts = [], []
    window = deque(maxlen=10)
    for twist in data.centroid_twists:
        if timestamp_to_float(twist.header) < t0:
            continue
        window.append(sqrt(twist.twist.linear.x**2 +
                           twist.twist.linear.y ** 2 + twist.twist.angular.z**2))
        sp.append(np.mean(window))
        ts.append(timestamp_to_float(twist.header) - t0)
    ax.plot(ts, sp, 'y', label='centroid')

    ax.set_title(f'Twists {data.filename.stem}')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('twist (m/s)')
    ax.set_ylim(0, 1)
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

        # fig = plot_path(data)
        # fig2 = plot_twist(data)
        # plot_x(data)
        # plot_twist_in_swarm(data)
        # plot_all_twist(data)

        print(data)
        get_metrics(data)
        plot_colored_path(data)

        plt.show()


if __name__ == "__main__":
    # main('rosbags/test2')
    # main('rosbags/rosbag2_2025_01_30-15_09_27')
    # main('rosbags/Experimentos/lineal/Lineal_Vel_05/rosbags/rosbag2_2025_01_24-12_49_36')
    main('rosbags/Experimentos/Curva/Curva_Vel_05/rosbags/rosbag2_2025_01_24-13_06_54')

    # main('rosbags/Experimentos/lineal/Lineal_Vel_05/rosbags')
    # main('rosbags/Experimentos/Lineal_Vel_1/')
    # main('rosbags/Experimentos/lineal/Lineal_Vel_2')
    # main('rosbags/Experimentos/Curva/Curva_Vel_05/rosbags')
    # main('rosbags/Experimentos/Curva/Curva_Vel_1')
    # main('rosbags/Experimentos/Curva/Curva_Vel_2')

    # main('rosbags/Experimentos/detach_drone')
