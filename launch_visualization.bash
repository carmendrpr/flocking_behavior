#!/bin/bash

# Get config file as argument. By default, use config/tf_visualization.rviz
config_file=${1:-"config/tf_visualization.rviz"}

# Launch rviz with the provided config file
rviz2 -d ${config_file}