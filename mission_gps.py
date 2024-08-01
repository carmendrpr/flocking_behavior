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

"""Simple mission GPS for a single drone."""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
from time import sleep

from as2_python_api.drone_interface_gps import DroneInterfaceGPS
import rclpy

TAKE_OFF_HEIGHT = 5.0  # Height in meters
TAKE_OFF_SPEED = 1.0  # Max speed in m/s
SLEEP_TIME = 0.5  # Sleep time between behaviors in seconds
SPEED = 1.0  # Max speed in m/s
HEIGHT = 5.0  # Height in meters
GPS_PATH = [
    [40.4405287, -3.6898277, HEIGHT],
    [40.4405298, -3.6898296, HEIGHT],
    [40.4405410, -3.6898235, HEIGHT],
    [40.4405914, -3.6898241, HEIGHT],
    [40.4405934, -3.6898216, HEIGHT],
    [40.4405068, -3.6898277, HEIGHT]
]
LAND_SPEED = 0.5  # Max speed in m/s


def drone_start(drone_interface: DroneInterfaceGPS) -> bool:
    """
    Take off the drone.

    :param drone_interface: DroneInterfaceGPS object
    :return: Bool indicating if the take off was successful
    """
    print('Start mission')

    # Arm
    print('Arm')
    success = drone_interface.arm()
    print(f'Arm success: {success}')

    # Offboard
    print('Offboard')
    success = drone_interface.offboard()
    print(f'Offboard success: {success}')

    # Take Off
    print('Take Off')
    success = drone_interface.takeoff(height=TAKE_OFF_HEIGHT, speed=TAKE_OFF_SPEED)
    print(f'Take Off success: {success}')

    return success


def drone_run(drone_interface: DroneInterfaceGPS) -> bool:
    """
    Run the mission for a single drone.

    :param drone_interface: DroneInterfaceGPS object
    :return: Bool indicating if the mission was successful
    """
    print('Run mission')

    # Go to path with keep yaw
    for goal in GPS_PATH:
        print(f'Go to with keep yaw {goal}')
        success = drone_interface.go_to.go_to_gps_point(goal, speed=SPEED)
        print(f'Go to success: {success}')
        if not success:
            return success
        print('Go to done')
        sleep(SLEEP_TIME)

    # Go to path facing
    for goal in GPS_PATH:
        print(f'Go to with path facing {goal}')
        success = drone_interface.go_to.go_to_gps_point_path_facing(goal, speed=SPEED)
        print(f'Go to success: {success}')
        if not success:
            return success
        print('Go to done')
        sleep(SLEEP_TIME)


def drone_end(drone_interface: DroneInterfaceGPS) -> bool:
    """
    End the mission for a single drone.

    :param drone_interface: DroneInterfaceGPS object
    :return: Bool indicating if the land was successful
    """
    print('End mission')

    # Land
    print('Land')
    success = drone_interface.land(speed=LAND_SPEED)
    print(f'Land success: {success}')
    if not success:
        return success

    # Manual
    print('Manual')
    success = drone_interface.manual()
    print(f'Manual success: {success}')

    return success


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

    print(f'Running mission for drone {drone_namespace}')

    rclpy.init()

    uav = DroneInterfaceGPS(
        drone_id=drone_namespace,
        use_sim_time=use_sim_time,
        verbose=verbosity)

    success = drone_start(uav)
    if success:
        success = drone_run(uav)
    success = drone_end(uav)

    uav.shutdown()
    rclpy.shutdown()
    print('Clean exit')
    exit(0)
