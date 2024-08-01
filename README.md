# Project AS2 Multirotor Simulator

Please refer to https://aerostack2.github.io/_02_examples/multirotor_simulator/project_multirotor_simulator/index.html for more information.

## Installation

To install this project, clone the repository:

```bash
git clone https://github.com/aerostack2/project_as2_multirotor_simulator.git
```

To start using this project, please go to the root folder of the project.

## Execution

### 1. Launch aerostack2 nodes for each drone
To launch aerostack2 nodes for each drone, execute once the following command:

```bash
./launch_as2.bash
```

The flags for the components launcher are:

- **-c**: motion controller plugin (pid_speed_controller, differential_flatness_controller), choices: [pid, df]. Default: pid
- **-m**: multi agent
- **-b**: launch behavior tree
- **-n**: drone namespaces, comma separated. Default get from world config file
- **-g**: launch using gnome-terminal instead of tmux

### 2. Launch aerostack2 nodes for the ground station
To launch aerostack2 nodes for the ground station, execute once the following command:

```bash
./launch_ground_station.bash
```

The flags for the components launcher are:

- **-m**: multi agent
- **-t**: launch keyboard teleoperation. Default not launch
- **-v**: open rviz. Default launch
- **-r**: record rosbag. Default not launch
- **-n**: drone namespaces, comma separated. Default get from world config file
- **-g**: launch using gnome-terminal instead of tmux

### 3. Launch a mission
There are several missions that can be executed:

- **AS2 keyboard teleoperation control**: You can use the keyboard teleoperation launched with the ground station, using the flag `-t`:
  ```bash
  ./launch_ground_station.bash -t
  ```
  You can launch a **swarm of drones** with the flag `-m` and control them with the keyboard teleoperation, as:
  ```bash
  ./launch_as2.bash -m
  ```
  ```bash
  ./launch_ground_station.bash -m -t
  ```
- **AS2 Python API single drone mission**: You can execute a mission that used AS2 Python API, launching the mission with:
  ```bash
  python3 mission.py
  ```
- **AS2 Python API single drone mission using GPS**: You can execute a mission that used AS2 Python API with GPS, launching the mission with:
  ```bash
  python3 mission_gps.py
  ```
- **AS2 Python API swarm of drones mission**: You can execute a mission with a swarm of drones that used AS2 Python API, launching the mission with:
  ```bash
  python3 mission_swarm.py
  ```
  You must launch a **swarm of drones** with the flag `-m`, as:
  ```bash
  ./launch_as2.bash -m
  ```
  ```bash
  ./launch_ground_station.bash -m
  ```
- **AS2 Mission Interpreter single drone mission**: You can execute a mission that used AS2 Mission Interpreter, launching the mission with:
  ```bash
  python3 mission_interpreter.py
  ```
- **AS2 Behavior Trees single drone mission**: You can execute a mission that used AS2 Behavior Trees, launching the mission with:
  ```bash
  python3 mission_behavior_tree.py
  ```
  You must launch behavior trees with the flag `-b`, as:
  ```bash
  ./launch_as2.bash -b
  ```

### 4. End the execution

If you are using tmux, you can end the execution with the following command:

- **End the execution of all nodes**:
  ```bash
  ./stop_tmuxinator_as2.bash
  ```
- **End the execution of all nodes of the ground station**:
  ```bash
  ./stop_tmuxinator_ground_station.bash
  ```
- **End the execution of both**:
  ```bash
  ./stop_tmuxinator.bash
  ```

You can force the end of all tmux sessions with the command:
```bash
tmux kill-server
```

If you are using gnome-terminal, you can end the execution by closing the terminal.


## Developers guide

All projects in aerostack2 are structured in the same way. The project is divided into the following directories:

- **tmuxinator**: Contains the tmuxinator launch file, which is used to launch all aerostack2 nodes.
  - **aerostack2.yaml**: Tmuxinator launch file for each drone. The list of nodes to be launched is defined here.
  - **ground_station.yaml**: Tmuxinator launch file for the ground station. The list of nodes to be launched is defined here.
- **config**: Contains the configuration files for the launchers of the nodes in the drones.
- **config_ground_station**: Contains the configuration files for the launchers of the nodes in the ground station.
- **launch_as2.bash**: Script to launch nodes defined in *tmuxinator/aerostack2.yaml*.
- **launch_ground_station.bash**: Script to launch nodes defined in *tmuxinator/ground_station.yaml*.
- **mission_\*.py**: Differents python mission files that can be executed.
- **stop_tmuxinator_as2.bash**: Script to stop all nodes launched by *launch_as2.bash*.
- **stop_tmuxinator_ground_station.bash**: Script to stop all nodes launched by *launch_ground_station.bash*.
- **stop_tmuxinator.bash**: Script to stop all nodes launched by *launch_as2.bash* and *launch_ground_station.bash*.
- **rosbag/record_rosbag.bash**: Script to record a rosbag. Can be modified to record only the topics that are needed.
- **trees\***: Contains the behavior trees that can be executed. They can be selected in the *aerostack2.yaml* file.
- **utils**: Contains utils scripts for launchers.

Both python and bash scripts have a help message that can be displayed by running the script with the `-h` option. For example, `./launch_as2.bash -h` will display the help message for the `launch_as2.bash` script.

**Note**: For knowing all parameters for each launch, you can execute the following command:

```bash
ros2 launch my_package my_launch.py -s
```

Also, you can see them in the default config file of the package, in the *config* folder. If you want to modify the default parameters, you can add the parameter to the config file.