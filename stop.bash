#!/bin/bash

# Get the directory of the current script
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Get drone namespaces from command-line argument
drones_namespace_comma=$1
IFS=',' read -r -a drone_namespaces <<< "$drones_namespace_comma"

# Make a tmux list of sessions to be killed
tmux_session_list=("rosbag")

# Add drones from user input
for namespace in ${drone_namespaces[@]}; do
  tmux_session_list+=("$namespace")
done

# Add drones from config/world.yaml file
drone_namespaces=$(python3 ${script_dir}/utils/get_drones.py -p ${script_dir}/config/config.yaml --sep ' ')
for namespace in ${drone_namespaces[@]}; do
  tmux_session_list+=("$namespace")
done

tmux_session_list+=("ground_station")

${script_dir}/utils/stop_tmux_sessions.bash "${tmux_session_list[@]}"
