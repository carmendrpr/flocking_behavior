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

__authors__ = 'Pedro Arias Pérez & Carmen De Rojas Pita-Romero'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


import argparse
import sys
from typing import List, Optional
import rclpy
from as2_msgs.msg import YawMode
from as2_msgs.msg import BehaviorStatus
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from as2_msgs.msg import PoseWithID
from geometry_msgs.msg import PoseStamped


class Drone(DroneInterface):
    """Drone Interface extended with async behavior wait"""

    def __init__(self, namespace: str, verbose: bool = False,
                 use_sim_time: bool = False):
        super().__init__(namespace, verbose=verbose, use_sim_time=use_sim_time)

        self.__speed = 1.0
        self.__yaw_mode = YawMode.PATH_FACING
        self.__yaw_angle = None
        self.__frame_id = "earth"

        self.current_behavior: Optional[BehaviorHandler] = None

    def do_behavior(self, beh, *args) -> None:
        """Start behavior and save current to check if finished or not"""
        self.current_behavior = getattr(self, beh)
        self.current_behavior(*args)

    def go_to(self, point) -> None:
        """Got to position"""
        self.do_behavior("go_to", point[0], point[1], point[2], self.__speed,
                         self.__yaw_mode, self.__yaw_angle, self.__frame_id, False)

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
        self.drones: dict[int, Drone] = {}
        self.drones_ns = drones_ns
        for index, name in enumerate(drones_ns):
            self.drones[index] = Drone(name, verbose, use_sim_time)

        self.drones[0].load_module("flocking")

    def shutdown(self):
        """Shutdown all drones in swarm"""
        for drone in self.drones.values():
            drone.shutdown()

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

    def run(self, virtual_centroid: tuple[str, List[float]], swarm_formation: list[tuple[str, list[float]]], drones_namespace: list[str], wait=True) -> None:
        self.drones[0].flocking(virtual_centroid, swarm_formation, self.drones_ns, wait)

    def modify(self, virtual_centroid: tuple[str, List[float]], swarm_formation: list[tuple[str, list[float]]], drones_namespace: list[str]) -> None:
        self.drones[0].flocking.modify_swarm_srv(
            virtual_centroid, swarm_formation, drones_namespace)

    def pause(self):
        self.drones[0].flocking.pause()

    def resume(self):
        self.drones[0].flocking.resume()

    def land_one_drone(self, drone_id):
        """Land one drone"""
        drone = self.drones[drone_id]
        drone.do_behavior("land", 0.4, False)

    def takeoff_one_drone(self, dron_id):
        """Takeoff one drone"""
        drone = self.drones[dron_id]
        drone.do_behavior("takeoff", 1, 0.7, False)


def confirm(msg: str = 'Continue') -> bool:
    """Confirm message"""
    confirmation = input(f"{msg}? (y/n): ")
    if confirmation == "y":
        return True
    return False


def pose(id, x, y, z) -> PoseWithID:
    pose = PoseWithID()
    pose.id = str(id)
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    return pose


def centroid(frame, px, py, pz, ox, oy, oz, ow) -> PoseStamped:
    virtual_centroid = PoseStamped()
    virtual_centroid.header.frame_id = str(frame)
    virtual_centroid.pose.position.x = float(px)
    virtual_centroid.pose.position.y = float(py)
    virtual_centroid.pose.position.z = float(pz)
    virtual_centroid.pose.orientation.x = float(ox)
    virtual_centroid.pose.orientation.y = float(oy)
    virtual_centroid.pose.orientation.z = float(oz)
    virtual_centroid.pose.orientation.w = float(ow)
    return virtual_centroid


def main():
    import time
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
    time.sleep(2)
    swarm.get_ready()
    swarm.takeoff()
    time.sleep(2)

    """ Define the Swarm Formation"""
    virtual_centroid = centroid("world", 1.0, 2.0, 1.5, 0.0, 0.0, 0.0, 1.0)
    drone0 = pose("drone0", -1.0, 1.0, 0.0)
    drone1 = pose("drone1", 1.41, 0.0, 0.0)
    drone2 = pose("drone2", -1.0, -1.0, 0.0)
    swarm_formation = list[PoseWithID]
    swarm_formation = [drone0, drone1, drone2]
    new_virtual_centroid = centroid("world", 3.0, 3.0, 1.5, 0.0, 0.0, 0.0, 1.0)
    new_drone2 = pose("drone2", -2.0, -2.0, 0.0)
    new_swarm_formation = list[PoseWithID]
    new_swarm_formation = [drone0, drone1, new_drone2]
    new_second_swarm_formation = list[PoseWithID]
    new_second_swarm_formation = [drone0, drone1]
    new_ns = list[str]
    new_ns = ['drone0', 'drone1']

    """ Send the action"""
    swarm.run(virtual_centroid, swarm_formation, swarm.drones_ns, False)

    time.sleep(12)

    """ Modify the virtual centroid"""
    swarm.modify(new_virtual_centroid, swarm_formation, swarm.drones_ns)
    time.sleep(12)

    """ Modify the swarm formation"""
    swarm.modify(new_virtual_centroid, new_swarm_formation, swarm.drones_ns)
    time.sleep(12)

    """ Modify number of drones"""
    swarm.modify(new_virtual_centroid, new_second_swarm_formation, new_ns)
    time.sleep(1)
    swarm.land_one_drone(2)
    time.sleep(2)

    """ Pause and resume the swarm"""
    swarm.pause()
    time.sleep(2)
    swarm.resume()
    time.sleep(7)
    swarm.land()

    print("Shutdown")
    swarm.shutdown()
    rclpy.shutdown()

    sys.exit(0)


if __name__ == '__main__':
    main()
