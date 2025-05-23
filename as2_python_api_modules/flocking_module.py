"""Flocking module."""

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

import os
import typing
import sys

# Append mission to path
sys.path.append(os.path.dirname(__file__))  # noqa


from as2_msgs.msg import YawMode, PoseWithID
from flocking_behavior import FlockingBehavior
from as2_python_api.modules.module_base import ModuleBase
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

if typing.TYPE_CHECKING:
    from as2_python_api.drone_interface import DroneInterface


class FlockingModule(ModuleBase, FlockingBehavior):
    """Flocking Module."""

    __alias__ = 'flocking'

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)

    def __call__(self, virtual_centroid: PoseStamped, swarm_formation: list[PoseWithID], drones_namespace: list[str],
                 wait: bool = True) -> bool:
        """
        Flocking.

        :param path: path to follow
        :type path: Path
        :param speed: speed (m/s) limit
        :type speed: float
        :param yaw_mode: yaw mode, defaults to YawMode.KEEP_YAW
        :type yaw_mode: int, optional
        :param yaw_angle: yaw angle (rad) when fixed yaw is set, defaults to None
        :type yaw_angle: float, optional
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :param wait: blocking call, defaults to True
        :type wait: bool, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.__flocking(virtual_centroid, swarm_formation, drones_namespace, wait)

    def __flocking(self, virtual_centroid: PoseStamped, swarm_formation: list[PoseWithID], drones_namespace: list[str],
                   wait: bool = True) -> bool:
        """
        Flocking.

        :param path: path to follow
        :type path: Path
        :param speed: speed (m/s) limit
        :type speed: float
        :param yaw_mode: yaw mode
        :type yaw_mode: int
        :param yaw_angle: yaw angle (rad) when fixed yaw is set
        :type yaw_angle: float
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :type frame_id: str, optional
        :param wait: blocking call, defaults to True
        :type wait: bool, optional
        :return: True if was accepted, False otherwise
        :rtype: bool
        """
        return self.start(virtual_centroid=virtual_centroid, swarm_formation=swarm_formation, drones_namespace=drones_namespace, wait_result=wait)
