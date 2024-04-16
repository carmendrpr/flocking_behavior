#!/bin/python3

"""
mission_planner.py
"""

import rclpy

from as2_python_api.mission_interpreter.mission import Mission
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter


def main():
    """Set a mission plan and execute it."""

    mission_json = """
    {
        "target": "drone0",
        "verbose": "False",
        "use_sim_time": "True",
        "plan": [
            {
                "behavior": "takeoff", 
                "args": {
                    "height": 1.0,
                    "speed": 1.0
                }
            },
            {
                "behavior": "go_to", 
                "args": {
                    "_x": 5.0,
                    "_y": 5.0,
                    "_z": 1.0,
                    "speed": 0.5,
                    "yaw_mode": 1
                }
            },
            {
                "behavior": "go_to", 
                "args": {
                    "_x": 5.0,
                    "_y": -5.0,
                    "_z": 1.0,
                    "speed": 0.5,
                    "yaw_mode": 1
                }
            },
            {
                "behavior": "go_to", 
                "args": {
                    "_x": -5.0,
                    "_y": 5.0,
                    "_z": 1.0,
                    "speed": 0.5,
                    "yaw_mode": 1
                }
            },
            {
                "behavior": "go_to", 
                "args": {
                    "_x": -5.0,
                    "_y": -5.0,
                    "_z": 1.0,
                    "speed": 0.5,
                    "yaw_mode": 1
                }
            },
            {
                "behavior": "go_to", 
                "args": {
                    "_x": 0.0,
                    "_y": 0.0,
                    "_z": 1.0,
                    "speed": 0.5,
                    "yaw_mode": 1
                }
            },
            {
                "behavior": "land", 
                "args": {
                    "speed": 0.5
                }
            }
        ]
    }
    """
    mission = Mission.parse_raw(mission_json)
    print(mission)

    rclpy.init()
    interpreter = MissionInterpreter(mission=mission)
    print("Start mission!")

    interpreter.drone.arm()
    interpreter.drone.offboard()
    interpreter.perform_mission(debug=True)

    print("Mission completed!")
    interpreter.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
