#!/bin/bash

usage() {
    echo "  Add drones namespaces as arguments, separated by commas"
}

# Get drone namespaces from command-line argument
input=$1

# Get the directory of the current script
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Dettach from the current tmux session if inside one
if [ -n "$TMUX" ]; then
    tmux detach
fi

# Start each script in its own tmux session
tmux new-session -d -s ground_station_stop "${script_dir}/stop_tmuxinator_ground_station.bash"
tmux new-session -d -s as2_stop "${script_dir}/stop_tmuxinator_as2.bash" "$input"

sleep 2

# Kill the tmux sessions
tmux kill-session -t ground_station_stop
tmux kill-session -t as2_stop