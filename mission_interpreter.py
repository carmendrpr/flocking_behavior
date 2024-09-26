#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Simple mission for a single drone using the mission interpreter."""

__authors__ = 'Rafael Perez-Segui, Pedro Arias-Perez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import json

import rclpy

from as2_python_api.mission_interpreter.mission import Mission
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter

TAKE_OFF_HEIGHT = 1.0  # Height in meters
TAKE_OFF_SPEED = 1.0  # Max speed in m/s
SLEEP_TIME = 0.5  # Sleep time between behaviors in seconds
SPEED = 1.0  # Max speed in m/s
HEIGHT = 1.0  # Height in meters
DIM = 2.0
PATH = [
    [-DIM, DIM, HEIGHT],
    [-DIM, -DIM, HEIGHT],
    [DIM, -DIM, HEIGHT],
    [DIM, DIM, HEIGHT]
]
LAND_SPEED = 0.5  # Max speed in m/s


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Single drone mission')

    parser.add_argument('-n', '--namespace',
                        type=str,
                        default='drone0',
                        help='ID of the drone to be used in the mission')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')
    parser.add_argument('-s', '--use_sim_time',
                        action='store_true',
                        default=False,
                        help='Use simulation time')

    args = parser.parse_args()
    drone_namespace = args.namespace
    verbosity = args.verbose
    use_sim_time = args.use_sim_time

    mission_json = f"""
    {{
        "target": "{drone_namespace}",
        "verbose": "{verbosity}",
        "use_sim_time": "{use_sim_time}",
        "plan": [
            {{
                "behavior": "takeoff", 
                "args": {{
                    "height": {TAKE_OFF_HEIGHT},
                    "speed": {TAKE_OFF_SPEED}
                }}
            }},
            {{
                "behavior": "go_to", 
                "args": {{
                    "x": {PATH[0][0]},
                    "y": {PATH[0][1]},
                    "z": {PATH[0][2]},
                    "speed": {SPEED},
                    "yaw_mode": 1
                }}
            }},
            {{
                "behavior": "go_to", 
                "args": {{
                    "x": {PATH[1][0]},
                    "y": {PATH[1][1]},
                    "z": {PATH[1][2]},
                    "speed": {SPEED},
                    "yaw_mode": 1
                }}
            }},
            {{
                "behavior": "go_to", 
                "args": {{
                    "x": {PATH[2][0]},
                    "y": {PATH[2][1]},
                    "z": {PATH[2][2]},
                    "speed": {SPEED},
                    "yaw_mode": 1
                }}
            }},
            {{
                "behavior": "go_to", 
                "args": {{
                    "x": {PATH[3][0]},
                    "y": {PATH[3][1]},
                    "z": {PATH[3][2]},
                    "speed": {SPEED},
                    "yaw_mode": 1
                }}
            }},
            {{
                "behavior": "land", 
                "args": {{
                    "speed": {LAND_SPEED}
                }}
            }}
        ]
    }}
    """

    mission = Mission.parse_raw(mission_json)

    print(f"Mission to be executed: {mission}")

    rclpy.init()

    interpreter = MissionInterpreter(
        mission=mission,
        use_sim_time=use_sim_time)
    
    print('Start mission')
    interpreter.drone.arm()
    interpreter.drone.offboard()
    print('Run mission')
    interpreter.perform_mission()

    print("Mission completed")
    interpreter.shutdown()
    rclpy.shutdown()
    print('Clean exit')
    exit(0)
