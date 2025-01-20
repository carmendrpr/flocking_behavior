# Project AS2 Swarm Control

Please refer to  for more information.

## Installation

To install this project, clone the repository:

```bash
git clone git@github.com:carmendrpr/project_swarm_control.git
```

To start using this project, please go to the root folder of the project.

## Execution

### 1. Launch aerostack2 nodes for three drone
To launch aerostack2 nodes for each drone, execute once the following command:

```bash
./launch_as2.bash -m
```
- **-m**: multi agent. Default not set


### 2. Launch the mission for three drones

```bash
python3 mission-swarm.py
```

### 4. End the execution

If you are using tmux, you can end the execution with the following command:

```bash
./stop.bash
```

You can force the end of all tmux sessions with the command:
```bash
tmux kill-server
```

