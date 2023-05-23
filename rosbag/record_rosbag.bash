#!/bin/bash

source utils/tools.bash

# Get drone namespaces from command-line argument
drone_namespaces_string=$1
drone_namespaces=($(string_to_list "$drone_namespaces_string"))

echo "Recording rosbag for drones: ${drone_namespaces[@]}"

# Create directory for rosbags
mkdir rosbag/rosbags 2>/dev/null
cd rosbag/rosbags

# Construct the rosbag record command
rosbag_cmd="ros2 bag record"

# Add topics and drone namespaces to the rosbag record command
for drone_namespace in "${drone_namespaces[@]}"; do
  rosbag_cmd+=" /${drone_namespace}/platform/info \
                /${drone_namespace}/self_localization/pose \
                /${drone_namespace}/self_localization/twist \
                /${drone_namespace}/actuator_command/twist \
                /${drone_namespace}/sensor_measurements/gps \
                /${drone_namespace}/sensor_measurements/odom"
done

# Add remaining topics
rosbag_cmd+=" /tf /tf_static --include-hidden-topics"

# Execute the rosbag record command
eval "$rosbag_cmd"