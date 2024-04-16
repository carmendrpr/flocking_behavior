#!/bin/python3

from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface


def drone_run(drone_interface: DroneInterface):
    """ Run the mission """

    takeoff_height = 3.0
    height = 3.0
    speed = 1.0
    sleep_time = 0.5
    dim = 2.0
    path = [
        [dim, dim, height],
        [dim, -dim, height],
        [-dim, dim, height],
        [-dim, -dim, height],
        [0.0, 0.0, takeoff_height],
    ]

    print("Start mission")

    ##### ARM OFFBOARD #####
    print("Arm")
    drone_interface.offboard()
    sleep(sleep_time)
    print("Offboard")
    drone_interface.arm()
    sleep(sleep_time)

    ##### TAKE OFF #####
    print("Take Off")
    drone_interface.takeoff(takeoff_height, speed=0.5)
    print("Take Off done")
    sleep(sleep_time)

    ##### GO TO #####
    for goal in path:
        print(f"Go to with keep yaw {goal}")
        drone_interface.go_to.go_to_point(goal, speed=speed)
        print("Go to done")
    sleep(sleep_time)

    for goal in path:
        print(f"Go to with path facing {goal}")
        drone_interface.go_to.go_to_point_path_facing(goal, speed=speed)
        print("Go to done")
    sleep(sleep_time)

    ##### FOLLOW PATH #####
    print("Follow path with keep yaw")
    drone_interface.follow_path.follow_path_with_keep_yaw(goal, speed=speed)
    print("Follow path")
    sleep(sleep_time)

    print("Follow path with path facing")
    drone_interface.follow_path.follow_path_with_path_facing(
        goal, speed=speed)
    print("Follow path")
    sleep(sleep_time)

    ##### LAND #####
    print("Landing")
    drone_interface.land(speed=0.3)
    print("Land done")

    drone_interface.disarm()


if __name__ == '__main__':

    rclpy.init()

    uav = DroneInterface(drone_id="drone0", verbose=False)

    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
