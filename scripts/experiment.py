"""
experiment.py
"""

from dataclasses import dataclass, field
from pathlib import Path
import matplotlib.pyplot as plt

from bag_analyzer import LogData, timestamp_to_float


@dataclass
class Stats:
    """ Stats """
    cohesion: dict[str, tuple[float, float]]
    separation: dict[str, tuple[float, float]]
    alignment: dict[str, tuple[float, float]]
    ref_error: dict[str, tuple[float, float]]

    def __str__(self) -> str:
        c = '------- COHESION -------\n'
        for k, v in self.cohesion.items():
            c += f'\t{k}: {v[0]:.3f} ± {v[1]:.3f} [m]\n'

        s = '------- SEPARATION -------\n'
        for k, v in self.separation.items():
            s += f'\t{k}: {v[0]:.3f} ± {v[1]:.3f} [m]\n'

        a = '------- ALIGNMENT -------\n'
        for k, v in self.alignment.items():
            a += f'\t{k}: {v[0]:.3f} ± {v[1]:.3f} [m/s]\n'

        r = '------- REFERENCE ERROR -------\n'
        for k, v in self.ref_error.items():
            r += f'\t{k}: {v[0]:.3f} ± {v[1]:.3f} [m]\n'

        return (f"{c}\n"
                f"{s}\n"
                f"{a}\n"
                f"{r}\n")


@dataclass
class Experiment:
    """ Experiment. Contains multiple rosbags for the same experiment. """
    name: str
    bags: list[str]

    # POST INIT
    log_datas: dict[str, LogData] = field(init=False)

    def __post_init__(self):
        experiments: dict[str, LogData] = {}

        for rosbag in self.bags:
            data = LogData.from_rosbag(Path(rosbag))
            experiments[rosbag] = data

        self.log_datas = experiments

    def __repr__(self) -> str:
        return f"Experiment {self.name} with {len(self.bags)} rosbags"

    def print_data_info(self):
        """ Print data info """
        print(f"\nData Info for {self.name}")
        for data in self.log_datas.values():
            print(data)

    def print_stats(self):
        """ Print stats """
        print(f"\nStats for {self.name}")
        print(self.stats)

    @property
    def stats(self):
        """ Statistics """
        cohesion = {}
        separation = {}
        alignment = {}
        ref_error = {}

        i = 0
        for v in self.log_datas.values():
            for k, v2 in v.cohesion_metric(timestamp_to_float(v.traj[0].header)).items():
                try:
                    cohesion[k][0] += v2[0]
                    cohesion[k][1] += v2[1]
                except KeyError:
                    cohesion[k] = list(v2)

            for k, v2 in v.separation_metric(timestamp_to_float(v.traj[0].header)).items():
                try:
                    separation[k][0] += v2[0]
                    separation[k][1] += v2[1]
                except KeyError:
                    separation[k] = list(v2)

            for k, v2 in v.alignment_metric(timestamp_to_float(v.traj[0].header)).items():
                try:
                    alignment[k][0] += v2[0]
                    alignment[k][1] += v2[1]
                except KeyError:
                    alignment[k] = list(v2)
            for k, v2 in v.ref_error_metric(timestamp_to_float(v.traj[0].header)).items():
                try:
                    ref_error[k][0] += v2[0]
                    ref_error[k][1] += v2[1]
                except KeyError:
                    ref_error[k] = list(v2)
            i += 1
        cohesion = {k: (v[0] / i, v[1] / i) for k, v in cohesion.items()}
        separation = {k: (v[0] / i, v[1] / i) for k, v in separation.items()}
        alignment = {k: (v[0] / i, v[1] / i) for k, v in alignment.items()}
        ref_error = {k: (v[0] / i, v[1] / i) for k, v in ref_error.items()}
        return Stats(cohesion, separation, alignment, ref_error)

    def plot_path(self):
        """Plot paths"""
        colors = ['r', 'm', 'b', 'y', 'g', 'c', 'k', 'w']
        fig, ax = plt.subplots()
        i = 0
        for data in self.log_datas.values():
            for drone, poses, c in zip(data.poses.keys(), data.poses.values(), colors):
                # https://stackoverflow.com/questions/52773215
                x = [pose.pose.position.x for pose in poses]
                y = [pose.pose.position.y for pose in poses]
                ax.plot(x, y, c, label=f'{drone}_{i}')
            i += 1
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


LINEAL05 = [
    'rosbags/Experimentos/lineal/Lineal_Vel_05/rosbags/rosbag2_2025_01_24-12_49_36',
    'rosbags/Experimentos/lineal/Lineal_Vel_05/rosbags/rosbag2_2025_01_24-12_53_06',
    'rosbags/Experimentos/lineal/Lineal_Vel_05/rosbags/rosbag2_2025_01_24-12_54_51',
    'rosbags/Experimentos/lineal/Lineal_Vel_05/rosbags/rosbag2_2025_01_24-13_00_06',
    'rosbags/Experimentos/lineal/Lineal_Vel_05/rosbags/rosbag2_2025_01_24-13_03_35',
]

LINEAL1 = []

LINEAL2 = [
    'rosbags/Experimentos/lineal/Lineal_Vel_2/rosbag2_2025_01_27-10_16_53',
    'rosbags/Experimentos/lineal/Lineal_Vel_2/rosbag2_2025_01_27-10_25_33',
    'rosbags/Experimentos/lineal/Lineal_Vel_2/rosbag2_2025_01_27-10_29_01',
    'rosbags/Experimentos/lineal/Lineal_Vel_2/rosbag2_2025_01_27-10_32_28',
]


CURVA05 = [
    'rosbags/Experimentos/Curva/Curva_Vel_05/rosbags/rosbag2_2025_01_24-13_06_54',
    'rosbags/Experimentos/Curva/Curva_Vel_05/rosbags/rosbag2_2025_01_24-13_10_24',
    'rosbags/Experimentos/Curva/Curva_Vel_05/rosbags/rosbag2_2025_01_24-13_12_09',
    'rosbags/Experimentos/Curva/Curva_Vel_05/rosbags/rosbag2_2025_01_24-13_13_54',
    'rosbags/Experimentos/Curva/Curva_Vel_05/rosbags/rosbag2_2025_01_24-13_15_39',
    'rosbags/Experimentos/Curva/Curva_Vel_05/rosbags/rosbag2_2025_01_24-13_17_25',
    'rosbags/Experimentos/Curva/Curva_Vel_05/rosbags/rosbag2_2025_01_24-13_19_10',
    'rosbags/Experimentos/Curva/Curva_Vel_05/rosbags/rosbag2_2025_01_24-13_22_39',
]

CURVA1 = [
    'rosbags/Experimentos/Curva/Curva_Vel_1/rosbag2_2025_01_27-09_24_02',
    'rosbags/Experimentos/Curva/Curva_Vel_1/rosbag2_2025_01_27-09_30_57',
    # 'rosbags/Experimentos/Curva/Curva_Vel_1/rosbag2_2025_01_27-09_32_41',
    'rosbags/Experimentos/Curva/Curva_Vel_1/rosbag2_2025_01_27-09_34_25',
]

CURVA2 = [
    'rosbags/Experimentos/Curva/Curva_Vel_2/rosbag2_2025_01_27-09_55_10',
    'rosbags/Experimentos/Curva/Curva_Vel_2/rosbag2_2025_01_27-09_56_54',
    'rosbags/Experimentos/Curva/Curva_Vel_2/rosbag2_2025_01_27-10_02_06',
    'rosbags/Experimentos/Curva/Curva_Vel_2/rosbag2_2025_01_27-10_05_33',
    'rosbags/Experimentos/Curva/Curva_Vel_2/rosbag2_2025_01_27-10_07_17',
    'rosbags/Experimentos/Curva/Curva_Vel_2/rosbag2_2025_01_27-10_09_01',
]

if __name__ == "__main__":
    # exp = Experiment("LINEAL05", LINEAL05)
    # exp.print_stats()

    exp = Experiment("CURVA1", CURVA1)
    exp.print_stats()
    exp.plot_path()
    plt.show()
