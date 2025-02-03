#!/bin/bash

usage() {
    echo "  options:"
    echo "      -m: disable launch mocap4ros2. By default set."
    echo "      -t: launch keyboard teleoperation. Default not launch"
    echo "      -v: open rviz. Default launch"
    echo "      -r: record rosbag. Default not launch"
    echo "      -n: drone namespaces, comma separated. Default get from world description config file"
    echo "      -g: launch using gnome-terminal instead of tmux. Default not set"
}

# Initialize variables with default values
mocap4ros2="true"
keyboard_teleop="false"
rviz="true"
rosbag="false"
drones_namespace_comma=""
use_gnome="false"

# Parse command line arguments
while getopts "mtvrn:g" opt; do
  case ${opt} in
    m )
      mocap4ros2="false"
      ;;
    t )
      keyboard_teleop="true"
      ;;
    v )
      rviz="false"
      ;;
    r )
      rosbag="true"
      ;;
    n )
      drones_namespace_comma="${OPTARG}"
      ;;
    g )
      use_gnome="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[swrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# If no drone namespaces are provided, get them from the world description config file
if [ -z "$drones_namespace_comma" ]; then
  drones_namespace_comma=$(python3 utils/get_drones.py -p config/config.yaml --sep ',')
fi

# Select between tmux and gnome-terminal
tmuxinator_mode="start"
tmuxinator_end="wait"
tmp_file="/tmp/as2_project_launch_${drone_namespaces[@]}.txt"
if [[ ${use_gnome} == "true" ]]; then
  tmuxinator_mode="debug"
  tmuxinator_end="> ${tmp_file} && python3 utils/tmuxinator_to_genome.py -p ${tmp_file} && wait"
fi

# Launch aerostack2 ground station
eval "tmuxinator ${tmuxinator_mode} -n ground_station -p tmuxinator/ground_station.yaml \
  drone_namespace=${drones_namespace_comma} \
  keyboard_teleop=${keyboard_teleop} \
  rviz=${rviz} \
  mocap4ros2=${mocap4ros2} \
  rosbag=${rosbag} \
  ${tmuxinator_end}"

# Attach to tmux session
if [[ ${use_gnome} == "false" ]]; then
  tmux attach-session -t ground_station
# If tmp_file exists, remove it
elif [[ -f ${tmp_file} ]]; then
  rm ${tmp_file}
fi