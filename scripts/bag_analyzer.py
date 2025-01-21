"""Analyze the experiment results"""
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np
import cv2
import matplotlib.pyplot as plt

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

from bag_reader import read_rosbag, deserialize_msgs


@dataclass
class LogData:
    """Data read from rosbag file"""
    filename: Path
    timestamps: list[float] = field(default_factory=list)
    poses: dict[str, list[tuple[float, float]]] = field(default_factory=dict)
    twists: dict[str, list[tuple[float, float]]] = field(default_factory=dict)

    @classmethod
    def from_rosbag(cls, rosbag: Path) -> 'LogData':
        """Read the rosbag"""
        log_data = cls(rosbag)
        rosbag_msgs = read_rosbag(str(rosbag))
        for topic, msgs in rosbag_msgs.items():
            if "pose" in topic:
                poses = deserialize_msgs(msgs, PointStamped)
                drone_id = topic.split("/")[1]
                ts0 = log_data.parse_timestamp(poses[0].header)
                # Not common ts for all drones
                log_data.timestamps = [log_data.parse_timestamp(
                    pose.header) - ts0 for pose in poses]
                log_data.poses[drone_id] = [(msg.point.x, msg.point.y)
                                            for msg in poses]
            if "twist" in topic:
                continue
            else:
                print(f"Unknown topic: {topic}")
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
    for k, v in data.poses.items():
        print(len(v))
        print(len(v[0]))
        ax.plot(data.timestamps, v, label=k)
    ax.set_title(f'Path length {data.filename.stem}')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('path length (m)')
    ax.legend()
    ax.grid()
    fig.savefig(f"/tmp/path_{data.filename.stem}.png")
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
        # print(data.stats(25.0))
    plt.show()


if __name__ == "__main__":
    main('rosbag/test1')
    # main('rosbags/exploration_20231129_140425')
