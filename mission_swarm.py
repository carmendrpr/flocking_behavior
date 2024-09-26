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

"""Simple mission for a swarm of drones."""

__authors__ = 'Rafael Perez-Segui, Miguel Fernandez-Cortizas'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


import argparse
import sys
from typing import List, Optional
from math import radians, cos, sin
from itertools import cycle, islice
import rclpy
from as2_msgs.msg import YawMode
from as2_msgs.msg import BehaviorStatus
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler


class Choreographer:
    """Simple Geometric Choreographer"""

    @staticmethod
    def delta_formation(base: float, height: float, orientation: float = 0.0, center: list = [0.0, 0.0]):
        """Triangle"""
        theta = radians(orientation)
        v0 = [-height * cos(theta) / 2.0 - base * sin(theta) / 2.0 + center[0],
              base * cos(theta) / 2.0 - height * sin(theta) / 2.0 + center[1]]
        v1 = [height * cos(theta) / 2.0 + center[0], height * sin(theta) / 2.0 + center[1]]
        v2 = [-height * cos(theta) / 2.0 + base * sin(theta) / 2.0 + center[0],
              -base * cos(theta) / 2.0 - height * sin(theta) / 2.0 + center[1]]
        return [v0, v1, v2]

    @staticmethod
    def line_formation(length: float, orientation: float = 0.0, center: list = [0.0, 0.0]):
        """Line"""
        theta = radians(orientation)
        l0 = [length * cos(theta) / 2.0 + center[1], length * sin(theta) / 2.0 + center[1]]
        l1 = [0.0 + center[1], 0.0 + center[1]]
        l2 = [-length * cos(theta) / 2.0 + center[1], -length * sin(theta) / 2.0 + center[1]]
        return [l0, l1, l2]

    @staticmethod
    def draw_waypoints(waypoints):
        """Debug"""
        import matplotlib.pyplot as plt

        print(waypoints)

        xaxys = []
        yaxys = []
        for wp in waypoints:
            xaxys.append(wp[0])
            yaxys.append(wp[1])
        plt.plot(xaxys, yaxys, 'o-b')
        plt.xlim(-3, 3)
        plt.ylim(-3, 3)
        plt.ylabel('some numbers')
        plt.show()

    @staticmethod
    def do_cycle(formation: list, index: int, height: int):
        """List to cycle with height"""
        return list(e + [height]
                    for e in list(islice(cycle(formation), 0+index, 3+index)))


class Dancer(DroneInterface):
    """Drone Interface extended with path to perform and async behavior wait"""

    def __init__(self, namespace: str, path: list, verbose: bool = False,
                 use_sim_time: bool = False):
        super().__init__(namespace, verbose=verbose, use_sim_time=use_sim_time)

        self.__path = path

        self.__current = 0

        self.__speed = 0.5
        self.__yaw_mode = YawMode.PATH_FACING
        self.__yaw_angle = None
        self.__frame_id = "earth"

        self.current_behavior: Optional[BehaviorHandler] = None

    def reset(self) -> None:
        """Set current waypoint in path to start point"""
        self.__current = 0

    def do_behavior(self, beh, *args) -> None:
        """Start behavior and save current to check if finished or not"""
        self.current_behavior = getattr(self, beh)
        self.current_behavior(*args)

    def go_to_next(self) -> None:
        """Got to next position in path"""
        point = self.__path[self.__current]
        self.do_behavior("go_to", point[0], point[1], point[2], self.__speed,
                         self.__yaw_mode, self.__yaw_angle, self.__frame_id, False)
        self.__current += 1

    def goal_reached(self) -> bool:
        """Check if current behavior has finished"""
        if not self.current_behavior:
            return False

        if self.current_behavior.status == BehaviorStatus.IDLE:
            return True
        return False


class SwarmConductor:
    """Swarm Conductor"""

    def __init__(self, drones_ns: List[str], verbose: bool = False,
                 use_sim_time: bool = False):
        self.drones: dict[int, Dancer] = {}
        for index, name in enumerate(drones_ns):
            path = get_path(index)
            self.drones[index] = Dancer(name, path, verbose, use_sim_time)

    def shutdown(self):
        """Shutdown all drones in swarm"""
        for drone in self.drones.values():
            drone.shutdown()

    def reset_point(self):
        """Reset path for all drones in swarm"""
        for drone in self.drones.values():
            drone.reset()

    def wait(self):
        """Wait until all drones has reached their goal (aka finished its behavior)"""
        all_finished = False
        while not all_finished:
            all_finished = True
            for drone in self.drones.values():
                all_finished = all_finished and drone.goal_reached()

    def get_ready(self) -> bool:
        """Arm and offboard for all drones in swarm"""
        success = True
        for drone in self.drones.values():
            # Arm
            success_arm = drone.arm()

            # Offboard
            success_offboard = drone.offboard()
            success = success and success_arm and success_offboard
        return success

    def takeoff(self):
        """Takeoff swarm and wait for all drones"""
        for drone in self.drones.values():
            drone.do_behavior("takeoff", 1, 0.7, False)
        self.wait()

    def land(self):
        """Land swarm and wait for all drones"""
        for drone in self.drones.values():
            drone.do_behavior("land", 0.4, False)
        self.wait()

    def dance(self):
        """Perform swarm choreography"""
        self.reset_point()
        for _ in range(len(get_path(0))):
            for drone in self.drones.values():
                drone.go_to_next()
            self.wait()


def get_path(i: int) -> list:
    """Path: initial, steps, final

    1   1           6       7           0
    2       2   5               8       1
    0   3           4       9           2

    """
    center = [0.0, 0.0]
    delta_frontward = Choreographer.delta_formation(3, 3, 0, center)
    delta_backward = Choreographer.delta_formation(3, 3, 180, center)
    line = Choreographer.line_formation(3, 180, center)

    h1 = 1.0
    h2 = 2.0
    h3 = 3.0
    line_formation = [line[i] + [h3]]
    return Choreographer.do_cycle(delta_frontward, i, h1) + \
        Choreographer.do_cycle(delta_backward, i, h2) + \
        Choreographer.do_cycle(delta_frontward, i, h3) + \
        line_formation


def confirm(msg: str = 'Continue') -> bool:
    """Confirm message"""
    confirmation = input(f"{msg}? (y/n): ")
    if confirmation == "y":
        return True
    return False


def main():
    parser = argparse.ArgumentParser(
        description='Single drone mission')

    parser.add_argument('-n', '--namespaces',
                        type=list,
                        default=['drone0', 'drone1', 'drone2'],
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
    drones_namespace = args.namespaces
    verbosity = args.verbose
    use_sim_time = args.use_sim_time

    rclpy.init()
    swarm = SwarmConductor(
        drones_namespace,
        verbose=verbosity,
        use_sim_time=use_sim_time)

    if confirm("Takeoff"):
        swarm.get_ready()
        swarm.takeoff()

        if confirm("Go to"):
            swarm.dance()

            while confirm("Replay"):
                swarm.dance()

        confirm("Land")
        swarm.land()

    print("Shutdown")
    swarm.shutdown()
    rclpy.shutdown()

    sys.exit(0)


if __name__ == '__main__':
    main()
