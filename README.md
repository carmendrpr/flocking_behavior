# project_template

# Launch guide

Parameters:
* -n namespace - namespace for the drone. Default, not specified and uses `config/world.yaml` configuration. If specified, it uses `config/platform_config_file.yaml` configuration.
* -m - multi agent mode. Default is disabled. If specified, it uses `config/world_swarm.yaml` configuration.
* -d - launch rviz visualization. If not specified, it does not launch rviz visualization. If specified, it launches rviz visualization with `config/tf_visualization.rviz` configuration.
* -e estimator type - estimator type. Default is `ground_truth`. Available options: `ground_truth`, `raw_odometry`, `raw_odometry_gps`. It uses configuration from `config/state_estimator*.yaml`.
* -r - record rosbag. Default is disabled. If specified, it records rosbag in `rosbag` directory.
* -t - launch keyboard teleoperation. Default is disabled. If specified, it launches keyboard teleoperation.

Examples:
* Launch single drone with `config/world.yaml` configuration
```bash
./launch_as2.bash
```
* Launch single drone with custon namespace and `config/platform_config_file.yaml` configuration
```bash
./launch_as2.bash -n <namespace>
```
* Launch multi agent mode with `config/world_swarm.yaml` configuration
```bash
./launch_as2.bash -m
```
* Launch rviz visualization with `config/tf_visualization.rviz` configuration
```bash
./launch_as2.bash -d
```
* Launch estimator with `raw_odometry` type
```bash
./launch_as2.bash -e raw_odometry
```
* Record rosbag
```bash
./launch_as2.bash -r
```
* Launch keyboard teleoperation
```bash
./launch_as2.bash -t
```