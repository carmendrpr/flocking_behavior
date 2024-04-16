#!/bin/bash

usage() {
    echo "  options:"
    echo "      -n: drone namespace, default use world config"
    echo "      -m: multi agent, default false"
    echo "      -d: enable rviz visualization"
    echo "      -e: estimator_type, choices: [raw_odometry, ground_truth, raw_odometry_gps]"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
}

# Arg parser
while getopts "n:mdrte:" opt; do
  case ${opt} in
    n )
      drone_namespace="${OPTARG}"
      ;;
    m )
      swarm="true"
      ;;
    d )
      rviz="true"
      ;;
    r )
      record_rosbag="true"
      ;;
    t )
      launch_keyboard_teleop="true"
      ;;
    e )
      estimator_plugin="${OPTARG}"
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


source utils/tools.bash

# Shift optional args
shift $((OPTIND -1))

## DEFAULTS
drone_namespace=${drone_namespace:=""}
swarm=${swarm:="false"}
rviz=${rviz:="false"}
estimator_plugin=${estimator_plugin:="ground_truth"}
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}

# If drone namespace is not provided, use world config
drone_namespaces=()
world_config="world.yaml"
if [[ ${drone_namespace} == "" ]]; then
  # If swarm is true, set world config to world_swarm.yaml, else world.yaml
  if [[ ${swarm} == "true" ]]; then
    world_config="world_swarm.yaml"
  fi
  # Get drone namespaces from world config
  drone_namespaces=$(python3 utils/get_drones.py config/${world_config})
  world_config="${world_config}"
else
  drone_namespaces=(${drone_namespace})
  world_config="dummy.yaml"
fi

# For drone in drone_namespaces, create tmux session
drone_namespaces_list=($(echo $drone_namespaces | tr ':' ' '))
for drone_ns in "${drone_namespaces_list[@]}"; do
  tmuxinator start -n ${drone_ns} -p tmuxinator/aerostack2.yml drone_namespace=${drone_ns} estimator_plugin=${estimator_plugin} world_config=${world_config} &
  wait
done

if [[ ${rviz} == "true" ]]; then
  tmuxinator start -n rviz -p tmuxinator/rviz.yml rviz_config_file="config/tf_visualization.rviz" &
  wait
fi

if [[ ${record_rosbag} == "true" ]]; then
  tmuxinator start -n rosbag -p tmuxinator/rosbag.yml drone_namespaces=${drone_namespaces} &
  wait
fi

if [[ ${launch_keyboard_teleop} == "true" ]]; then
  drone_namespaces_comma=$(echo $drone_namespaces | tr ':' ',')
  tmuxinator start -n keyboard_teleop -p tmuxinator/keyboard_teleop.yml simulation=false drone_namespaces=${drone_namespaces_comma} &
  wait
fi

tmux attach-session -t ${drone_namespaces_list[0]}:mission