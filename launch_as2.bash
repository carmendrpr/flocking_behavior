#!/bin/bash

usage() {
    echo "  options:"
    echo "      -r: record rosbag"
    echo "      -c: motion controller plugin (pid_speed_controller, differential_flatness_controller), choices: [pid, df]. Default: pid"
    echo "      -m: 12 agent. For mission_12"
    echo "      -n: select drones namespace to launch, values are comma separated. By default, it will get all drones from world description file"
    echo "      -g: launch using gnome-terminal instead of tmux. Default not set"
}

# Initialize variables with default values
motion_controller_plugin="pid"
swarm12="false"
drones_namespace_comma=""
use_gnome="false"

# Arg parser
while getopts "rcmn:g" opt; do
  case ${opt} in
    r )
      record_rosbag="true"
      ;;
    c )
      motion_controller_plugin="${OPTARG}"
      ;;
    m )
      swarm12="true"
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
      if [[ ! $OPTARG =~ ^[wrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# HOW TO INCLUDE MODULES FROM THE PROJECT
export AS2_MODULES_PATH=$AS2_MODULES_PATH:$(pwd)/as2_python_api_modules

 
## DEFAULTS
record_rosbag=${record_rosbag:="false"}

# Set simulation world description config file for 4 drones or 12 drones
if [[ ${swarm12} == "true" ]]; then
    simulation_config="config/world_swarm_12.yaml"
    as2_config_file="config/config.yaml"
    mission_file="mission_12"
    echo "$swarm12"
else
    simulation_config="config/world_swarm_3.yaml"
    as2_config_file="config/config.yaml"
    rviz_config="config_ground_station/rviz3_config.rviz"
    mission_file="mission_swarm"
    echo "Default configuration used"
fi


# If no drone namespaces are provided, get them from the world description config file
if [ -z "$drones_namespace_comma" ]; then
  drones_namespace_comma=$(python3 utils/get_drones.py -p ${simulation_config} --sep ',')
fi
IFS=',' read -r -a drone_namespaces <<< "$drones_namespace_comma"

# Check if motion controller plugins are valid
case ${motion_controller_plugin} in
  pid )
    motion_controller_plugin="pid_speed_controller"
    ;;
  df )
    motion_controller_plugin="differential_flatness_controller"
    ;;
  * )
    echo "Invalid motion controller plugin: ${motion_controller_plugin}" >&2
    usage
    exit 1
    ;;
esac

# Select between tmux and gnome-terminal
tmuxinator_mode="start"
tmuxinator_end="wait"
tmp_file="/tmp/as2_project_launch_${drone_namespaces[@]}.txt"
if [[ ${use_gnome} == "true" ]]; then
  tmuxinator_mode="debug"
  tmuxinator_end="> ${tmp_file} && python3 utils/tmuxinator_to_genome.py -p ${tmp_file} && wait"
fi

# Launch aerostack2 for each drone namespace
for namespace in ${drone_namespaces[@]}; do
  base_launch="false"
  if [[ ${namespace} == ${drone_namespaces[0]} ]]; then
    base_launch="true"
  fi
  eval "tmuxinator ${tmuxinator_mode} -n ${namespace} -p tmuxinator/aerostack2.yaml \
    drone_namespace=${namespace} \
    drone_namespace_list=${drones_namespace_comma} \
    simulation_config_file=${simulation_config} \
    motion_controller_plugin=${motion_controller_plugin} \
    as2_config_file=${as2_config_file} \
    rviz_config_file=${rviz_config}\
    mission_file=${mission_file} \
    base_launch=${base_launch} \
    ${tmuxinator_end}"

  sleep 0.1 # Wait for tmuxinator to finish
done

if [[ ${record_rosbag} == "true" ]]; then
    tmuxinator start -p tmuxinator/rosbag.yml \
        drones=${drones_namespace_comma} &
    wait
fi

# Attach to tmux session
if [[ ${use_gnome} == "false" ]]; then
  tmux attach-session -t ${drone_namespaces[0]}
# If tmp_file exists, remove it
elif [[ -f ${tmp_file} ]]; then
  rm ${tmp_file}
fi
