"""
bpaper_lineal_05.py
"""

import csv
from builtin_interfaces.msg import Time
from typing import Any
from dataclasses import dataclass, field
from math import sqrt
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header


@dataclass
class LogData:
    """Data read from rosbag file"""
    poses: dict[str, list[PoseStamped]] = field(default_factory=dict)
    twists: dict[str, list[TwistStamped]] = field(default_factory=dict)


def timestamp_to_float(header: Header) -> float:
    """Parse timestamp from header and convert float"""
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def time_to_index(time: float, msgs_stamped: list[Any]) -> int:
    """Get index from time"""
    for i, msg in enumerate(msgs_stamped):
        if timestamp_to_float(msg.header) >= time:
            return i
    return -1


def load_log_data_from_csv(filename: str) -> LogData:
    """Load LogData from a CSV file."""
    data = LogData()
    
    with open(filename, mode='r') as file:
        reader = csv.reader(file)
        headers = next(reader)  # Skip headers
        
        for row in reader:
            drone, time, x, y, speed = row
            # Create PoseStamped and TwistStamped objects based on the data
            pose = PoseStamped()
            
            # Convert the time into ROS 2 Time message
            time_msg = Time()
            time_float = float(time)
            time_msg.sec = int(time_float)
            time_msg.nanosec = int((time_float - time_msg.sec) * 1e9)
            pose.header.stamp = time_msg
            
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            
            twist = TwistStamped()
            twist.header.stamp = time_msg
            twist.twist.linear.x = float(speed)  # Just an example, adjust if needed
            
            # Store data in the LogData structure
            if drone not in data.poses:
                data.poses[drone] = []
                data.twists[drone] = []
            data.poses[drone].append(pose)
            data.twists[drone].append(twist)

    print(f"Data loaded from {filename}")
    return data


def plot_colored_path_from_csv(csv_filename: str, t0: float = 0.0, tf: float = None):
    """Plot paths colored with speed from CSV file."""
    # Load the LogData from the CSV file
    data = load_log_data_from_csv(csv_filename)
    
    # The rest of your plotting code remains unchanged
    fig, ax = plt.subplots()
    x_before, y_before = [], []
    x_after, y_after = [], []
    x_initial, y_initial = [], []
    
    ax.grid()

    for drone, poses, twists in zip(data.poses.keys(), data.poses.values(), data.twists.values()):
        x = [pose.pose.position.x for pose in poses]
        y = [pose.pose.position.y for pose in poses]
        c = [sqrt(twist.twist.linear.x**2 + twist.twist.linear.y ** 2 + twist.twist.angular.z**2)
             for twist in twists]
        x = x[:len(c)]
        y = y[:len(c)]
        c = c[:len(x)]

        i0 = time_to_index(timestamp_to_float(poses[0].header) + t0, poses)
        i9 = time_to_index(timestamp_to_float(poses[0].header) + tf, poses) if tf else len(x)
        x = x[i0:i9]
        y = y[i0:i9]
        c = c[i0:i9]

        x_initial.append(x[0])
        y_initial.append(y[0])
        x_before.append(x[len(x) // 2])
        y_before.append(y[len(y) // 2])
        x_after.append(x[-1])
        y_after.append(y[-1])

        ax.scatter(x, y, s=1, c=c, cmap='plasma')
        # ax.plot(x[0], y[0], 'kD')
        # ax.text(x[0], y[0] - 0.25, drone)

    fig.colorbar(ax.collections[0], ax=ax, label='speed (m/s)')
    # ax.add_patch(Polygon([(x_before[0], y_before[0]), (x_before[1], y_before[1]), (x_before[2],
    #              y_before[2])], alpha=0.2, facecolor="ForestGreen", edgecolor="green", linewidth=2, zorder=1))
    # ax.add_patch(Polygon([(x_after[0], y_after[0]), (x_after[1], y_after[1]), (x_after[2], y_after[2])],
    #              alpha=0.2, facecolor="ForestGreen", edgecolor="green", linewidth=2, zorder=2))
    # ax.add_patch(Polygon([(x_initial[0], y_initial[0]), (x_initial[1], y_initial[1]), (x_initial[2], y_initial[2])],
    #              alpha=0.2, facecolor="ForestGreen", edgecolor="green", linewidth=2, zorder=2))

    ax.set_title(f'Path from {csv_filename}')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

    return fig


def paper_lineal_05():
    fig = plot_colored_path_from_csv('data.csv', t0=30.0, tf=90.0)
    fig.set_size_inches(7, 4.0)
    ax = fig.get_axes()[0]
    ax.set_title('')
    ax.set_ylim(-2, 2)

    fig.savefig(f"base_figure.png", dpi=600)
    # plt.show()
    

if __name__ == "__main__":
    paper_lineal_05()