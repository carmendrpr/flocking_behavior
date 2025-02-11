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
    'rosbags/Experimentos/Linear/Linear05/rosbag2_2025_01_30-15_43_07',
    'rosbags/Experimentos/Linear/Linear05/rosbag2_2025_01_30-15_44_51',
    'rosbags/Experimentos/Linear/Linear05/rosbag2_2025_01_30-15_46_35',
    'rosbags/Experimentos/Linear/Linear05/rosbag2_2025_01_30-15_48_18',
    'rosbags/Experimentos/Linear/Linear05/rosbag2_2025_01_30-15_50_02',
    'rosbags/Experimentos/Linear/Linear05/rosbag2_2025_01_30-15_51_46',
    'rosbags/Experimentos/Linear/Linear05/rosbag2_2025_01_30-15_53_30',
    'rosbags/Experimentos/Linear/Linear05/rosbag2_2025_01_30-15_55_14',
    'rosbags/Experimentos/Linear/Linear05/rosbag2_2025_01_30-15_56_58',
    'rosbags/Experimentos/Linear/Linear05/rosbag2_2025_01_30-15_58_42',
]

LINEAL1 = ['rosbags/Experimentos/Linear/Linear1/rosbag2_2025_01_30-16_18_13',
           'rosbags/Experimentos/Linear/Linear1/rosbag2_2025_01_30-16_19_57',
           'rosbags/Experimentos/Linear/Linear1/rosbag2_2025_01_30-16_21_40',
           'rosbags/Experimentos/Linear/Linear1/rosbag2_2025_01_30-16_23_25',
           'rosbags/Experimentos/Linear/Linear1/rosbag2_2025_01_30-16_25_09',
           'rosbags/Experimentos/Linear/Linear1/rosbag2_2025_01_30-16_26_53',
           'rosbags/Experimentos/Linear/Linear1/rosbag2_2025_01_30-16_28_37',
           'rosbags/Experimentos/Linear/Linear1/rosbag2_2025_01_30-16_30_20',
           'rosbags/Experimentos/Linear/Linear1/rosbag2_2025_01_30-16_32_04',
           'rosbags/Experimentos/Linear/Linear1/rosbag2_2025_01_30-16_33_48',
           ]

LINEAL2 = [
    'rosbags/Experimentos/Linear/Linear2/rosbag2_2025_01_30-16_37_25',
    'rosbags/Experimentos/Linear/Linear2/rosbag2_2025_01_30-16_39_09',
    'rosbags/Experimentos/Linear/Linear2/rosbag2_2025_01_30-16_40_53',
    'rosbags/Experimentos/Linear/Linear2/rosbag2_2025_01_30-16_42_37',
    'rosbags/Experimentos/Linear/Linear2/rosbag2_2025_01_30-16_44_21',
    'rosbags/Experimentos/Linear/Linear2/rosbag2_2025_01_30-16_46_04',
    'rosbags/Experimentos/Linear/Linear2/rosbag2_2025_01_30-16_47_48',
    'rosbags/Experimentos/Linear/Linear2/rosbag2_2025_01_30-16_49_32',
    'rosbags/Experimentos/Linear/Linear2/rosbag2_2025_01_30-16_51_16',
    'rosbags/Experimentos/Linear/Linear2/rosbag2_2025_01_30-16_53_00',
]


CURVA05 = [
    'rosbags/Experimentos/Curvilinear/Curvilinear05/rosbag2_2025_01_30-16_56_10',
    'rosbags/Experimentos/Curvilinear/Curvilinear05/rosbag2_2025_01_30-16_57_54',
    'rosbags/Experimentos/Curvilinear/Curvilinear05/rosbag2_2025_01_30-16_59_38',
    'rosbags/Experimentos/Curvilinear/Curvilinear05/rosbag2_2025_01_30-17_01_22',
    'rosbags/Experimentos/Curvilinear/Curvilinear05/rosbag2_2025_01_30-17_03_06',
    'rosbags/Experimentos/Curvilinear/Curvilinear05/rosbag2_2025_01_30-17_04_51',
    'rosbags/Experimentos/Curvilinear/Curvilinear05/rosbag2_2025_01_30-17_06_35',
    'rosbags/Experimentos/Curvilinear/Curvilinear05/rosbag2_2025_01_30-17_08_18',
    'rosbags/Experimentos/Curvilinear/Curvilinear05/rosbag2_2025_01_30-17_10_02',
    'rosbags/Experimentos/Curvilinear/Curvilinear05/rosbag2_2025_01_30-17_11_46',


]

CURVA1 = [
    'rosbags/Experimentos/Curvilinear/Curvilinear1/rosbag2_2025_01_30-17_15_06',
    'rosbags/Experimentos/Curvilinear/Curvilinear1/rosbag2_2025_01_30-17_16_50',
    'rosbags/Experimentos/Curvilinear/Curvilinear1/rosbag2_2025_01_30-17_18_33',
    'rosbags/Experimentos/Curvilinear/Curvilinear1/rosbag2_2025_01_30-17_20_18',
    'rosbags/Experimentos/Curvilinear/Curvilinear1/rosbag2_2025_01_30-17_22_02',
    'rosbags/Experimentos/Curvilinear/Curvilinear1/rosbag2_2025_01_30-17_23_45',
    'rosbags/Experimentos/Curvilinear/Curvilinear1/rosbag2_2025_01_30-17_25_30',
    'rosbags/Experimentos/Curvilinear/Curvilinear1/rosbag2_2025_01_30-17_27_14',
    'rosbags/Experimentos/Curvilinear/Curvilinear1/rosbag2_2025_01_30-17_28_57',
    'rosbags/Experimentos/Curvilinear/Curvilinear1/rosbag2_2025_01_30-17_30_41',
]

CURVA2 = [
    'rosbags/Experimentos/Curvilinear/Curvilinear2/rosbag2_2025_01_30-17_32_43',
    'rosbags/Experimentos/Curvilinear/Curvilinear2/rosbag2_2025_01_30-17_34_27',
    'rosbags/Experimentos/Curvilinear/Curvilinear2/rosbag2_2025_01_30-17_36_11',
    'rosbags/Experimentos/Curvilinear/Curvilinear2/rosbag2_2025_01_30-17_37_55',
    'rosbags/Experimentos/Curvilinear/Curvilinear2/rosbag2_2025_01_30-17_39_39',
    'rosbags/Experimentos/Curvilinear/Curvilinear2/rosbag2_2025_01_30-17_41_23',
    'rosbags/Experimentos/Curvilinear/Curvilinear2/rosbag2_2025_01_30-17_43_06',
    'rosbags/Experimentos/Curvilinear/Curvilinear2/rosbag2_2025_01_30-17_44_50',
    'rosbags/Experimentos/Curvilinear/Curvilinear2/rosbag2_2025_01_30-17_46_34',
    'rosbags/Experimentos/Curvilinear/Curvilinear2/rosbag2_2025_01_30-17_48_18',
]

if __name__ == "__main__":
    exp = Experiment("LINEAL05", CURVA1)
    exp.print_stats()
    exp.plot_path()
    plt.show()
