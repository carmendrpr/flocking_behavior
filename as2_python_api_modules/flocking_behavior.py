"""Flocking Behavior."""

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
#    * Neither the name of the the copyright holder nor the names of its
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


__authors__ = 'Pedro Arias Pérez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from typing import TYPE_CHECKING, Union

from as2_msgs.action import SwarmFlocking
from as2_msgs.msg import PoseWithID
from geometry_msgs.msg import PoseStamped

from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from as2_python_api.tools.utils import path_to_list

if TYPE_CHECKING:
    from as2_python_api.drone_interface_base import DroneInterfaceBase


class FlockingBehavior(BehaviorHandler):
    """Flocking Behavior."""

    def __init__(self, drone: 'DroneInterfaceBase') -> None:
        self.__drone = drone

        try:
            super().__init__(drone, SwarmFlocking, '/Swarm/SwarmFlockingBehavior')
        except self.BehaviorNotAvailable as err:
            self.__drone.get_logger().warn(str(err))

        self.__modify_swarm_srv = self.__drone.create_client(
            SwarmFlocking.Impl.SendGoalService, '/Swarm/swarm_modify_srv')

    def start(self, virtual_centroid: PoseStamped, swarm_formation: list[PoseWithID], drones_namespace: list[str], wait_result: bool = True) -> bool:
        """Start behavior."""
        goal_msg = SwarmFlocking.Goal()
        goal_msg.virtual_centroid.header.stamp = self.__drone.get_clock().now().to_msg()
        goal_msg.virtual_centroid.header.frame_id = virtual_centroid.header.frame_id
        goal_msg.virtual_centroid.pose.position.x = virtual_centroid.pose.position.x
        goal_msg.virtual_centroid.pose.position.y = virtual_centroid.pose.position.y
        goal_msg.virtual_centroid.pose.position.z = virtual_centroid.pose.position.z
        goal_msg.virtual_centroid.pose.orientation.x = virtual_centroid.pose.orientation.x
        goal_msg.virtual_centroid.pose.orientation.y = virtual_centroid.pose.orientation.y
        goal_msg.virtual_centroid.pose.orientation.z = virtual_centroid.pose.orientation.z
        goal_msg.virtual_centroid.pose.orientation.w = virtual_centroid.pose.orientation.w
        goal_msg.swarm_formation = swarm_formation
        goal_msg.drones_namespace = drones_namespace

        try:
            return super().start(goal_msg, wait_result)
        except self.GoalRejected as err:
            self.__drone.get_logger().warn(str(err))
        return False

    def modify_swarm_srv(self, virtual_centroid: PoseStamped, swarm_formation: list[PoseWithID], drones_namespace: list[str]):
        msg = SwarmFlocking.Impl.SendGoalService.Request()
        msg.goal.virtual_centroid.header.stamp = self.__drone.get_clock().now().to_msg()
        msg.goal.virtual_centroid.header.frame_id = virtual_centroid.header.frame_id
        msg.goal.virtual_centroid.pose.position.x = virtual_centroid.pose.position.x
        msg.goal.virtual_centroid.pose.position.y = virtual_centroid.pose.position.y
        msg.goal.virtual_centroid.pose.position.z = virtual_centroid.pose.position.z
        msg.goal.virtual_centroid.pose.orientation.x = virtual_centroid.pose.orientation.x
        msg.goal.virtual_centroid.pose.orientation.y = virtual_centroid.pose.orientation.y
        msg.goal.virtual_centroid.pose.orientation.z = virtual_centroid.pose.orientation.z
        msg.goal.virtual_centroid.pose.orientation.w = virtual_centroid.pose.orientation.w
        msg.goal.swarm_formation = swarm_formation
        msg.goal.drones_namespace = drones_namespace

        if not self.__modify_swarm_srv.service_is_ready():
            self.__drone.get_logger().warn("Service not available")
            return False
        self.__modify_swarm_srv.call(msg)
        return True
