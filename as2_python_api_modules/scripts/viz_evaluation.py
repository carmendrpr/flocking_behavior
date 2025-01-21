"""Visualize the exploration results"""
import bisect
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np
import cv2
import matplotlib.pyplot as plt

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header


from bag_reader import read_rosbag, deserialize_msgs


def find_closest_index(a, x):
    """https://stackoverflow.com/a/56335408/9553849"""
    i = bisect.bisect_left(a, x)
    if i >= len(a):
        i = len(a) - 1
    elif i and a[i] - x > x - a[i - 1]:
        i = i - 1
    return (i, a[i])


@dataclass
class LogData:
    """Data read from rosbag file"""
    filename: Path
    timestamps: list[float] = field(default_factory=list)
    area_pct: list[float] = field(default_factory=list)
    area_m2: list[float] = field(default_factory=list)
    paths: dict[str, list[float]] = field(default_factory=dict)
    total_path: list[float] = field(default_factory=list)
    real_poses: dict[str, list[tuple[float, float]]] = field(default_factory=dict)
    reference_poses: dict[str, list[tuple[float, float]]] = field(default_factory=dict)

    @classmethod
    def from_rosbag(cls, rosbag: Path) -> 'LogData':
        """Read the rosbag"""
        log_data = cls(rosbag)
        rosbag_msgs = read_rosbag(str(rosbag))
        for topic, msgs in rosbag_msgs.items():
            if topic == "/map_server/map_filtered":
                grids = deserialize_msgs(msgs, OccupancyGrid)
                ts0 = log_data.parse_timestamp(grids[0].header)
                for grid in grids:
                    log_data.timestamps.append(
                        log_data.parse_timestamp(grid.header) - ts0)
                    area, pct = log_data.parse_grid(grid)
                    log_data.area_pct.append(pct)
                    log_data.area_m2.append(area)
                log_data.last_occ_grid = grids[-1]
            elif "path_length" in topic:
                path_length = deserialize_msgs(msgs, PointStamped)
                drone_id = topic.split("/")[1]
                log_data.paths[drone_id] = [msg.point.x for msg in path_length]
                if not log_data.total_path:
                    log_data.total_path = log_data.paths[drone_id]
                else:
                    log_data.total_path = np.add(
                        log_data.total_path, log_data.paths[drone_id]).tolist()
            elif "pose" in topic:
                poses = deserialize_msgs(msgs, PointStamped)
                drone_id = topic.split("/")[1]
                log_data.real_poses[drone_id] = [(msg.point.x, msg.point.y)
                                                 for msg in poses]
        return log_data

    def parse_timestamp(self, header: Header) -> float:
        """Parse timestamp from header"""
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def __str__(self):
        """Print stats"""
        text = f"{self.filename.stem}\n"
        text += f"{self.timestamps[-1]:8.2f}s "
        text += f"{self.area_m2[-1]:8.2f}m^2 ({self.area_pct[-1]:5.2f}%) "
        text += f"{self.total_path[-1]:8.2f}m\n"
        return text

    def stats(self, pct_step: float = 25.0):
        """Print stats"""
        text = "  %      Time[s]   Area[m2]    Path[m]\n"
        text += "--------------------------------------\n"
        pct = 0.0
        while pct <= 100.0:
            i, _ = find_closest_index(self.area_pct, pct)
            text += f"{pct:3.0f}% {self.timestamps[i]:10.2f} "
            text += f"{self.area_m2[i]:10.2f} {self.total_path[i]:10.2f}\n"
            pct += pct_step

        return text


def plot_area(data: LogData, fig: plt.Figure = None) -> plt.Figure:
    """Plot area graph"""
    if fig is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title('Exploration results')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('area (%)')
        ax.grid()

        ax2 = ax.twinx()
        ax2.set_ylabel('area (m^2)')
    else:
        ax = fig.axes[0]
        ax2 = fig.axes[1]

    ax.plot(data.timestamps, data.area_pct, label=data.filename.stem)
    ax2.plot(data.timestamps, data.area_m2)

    fig.savefig("/tmp/area.png")
    ax.legend()
    return fig


def plot_total_path(data: LogData, fig: plt.Figure = None) -> plt.Figure:
    """Plot paths"""
    if fig is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title('Path length')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('path length (m)')
        ax.grid()
    else:
        ax = fig.axes[0]

    ax.plot(data.timestamps, data.total_path, label=data.filename.stem)

    ax.legend()
    fig.savefig("/tmp/total_path.png")
    return fig


def plot_path(data: LogData):
    """Plot paths"""
    fig, ax = plt.subplots()
    for k, v in data.paths.items():
        ax.plot(data.timestamps, v, label=k)
    ax.set_title(f'Path length {data.filename.stem}')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('path length (m)')
    ax.legend()
    ax.grid()
    fig.savefig(f"/tmp/path_{data.filename.stem}.png")
    return fig


def plot_area_with_error(ts, v, error, label='', fig: plt.Figure = None) -> plt.Figure:
    """Plot area graph"""
    if fig is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        # ax.set_title('Exploration results')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('area (%)')
        ax.grid()

        ax2 = ax.twinx()
        ax2.set_ylabel('area (m^2)')
    else:
        ax = fig.axes[0]
        ax2 = fig.axes[1]

    ax.plot(ts, v, label=label)
    ax2.set_ylim(0, 60)  # Hardcoded
    # ax2.plot(ts, v[1])
    # ax.fill_between(ts, v[0] - error[0], v[0] + error[0], alpha=0.3)
    ax.fill_between(ts, v - error, v + error, alpha=0.3)

    ax.legend()
    fig.savefig("/tmp/area_avg.png")
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
        fig = plot_area(data, fig)
        fig2 = plot_total_path(data, fig2)

        # plot_path(data)
        # print(data.stats(25.0))
    plt.show()


if __name__ == "__main__":
    main('rosbags/')
