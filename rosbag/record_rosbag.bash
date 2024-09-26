#!/bin/bash

usage() {
    echo "  Add drones namespaces as arguments, separated by commas"
}

# Get drone namespaces from command-line argument
drones_namespace_comma=$1
drone_namespaces=$(echo $drones_namespace_comma | tr "," " ")

echo "Recording rosbag for drones: ${drones_namespace_comma[@]}"

# Create directory for rosbags
mkdir rosbag/rosbags 2>/dev/null
cd rosbag/rosbags

# Construct the rosbag record command
rosbag_cmd="ros2 bag record"

# Uncomment the following lines to record specific topics
# Add topics and drone namespaces to the rosbag record command
# for drone_namespace in ${drone_namespaces[@]}; do
#   rosbag_cmd+=" /${drone_namespace}/platform/info \
#                 /${drone_namespace}/self_localization/pose \
#                 /${drone_namespace}/self_localization/twist \
#                 /${drone_namespace}/actuator_command/twist"
# done
# Add remaining topics
# rosbag_cmd+=" /tf /tf_static"

# Comment the following line to record specific topics
# Record all topics
rosbag_cmd+=" --all"

# Include hidden topics
rosbag_cmd+="  --include-hidden-topics"

# Execute the rosbag record command
eval "$rosbag_cmd"