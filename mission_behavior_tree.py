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

"""Simple mission with Behavior Trees."""

__authors__ = 'Pedro Arias-Perez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String
from rclpy.parameter import Parameter


class StartBehaviorTree(Node):
    """Behavior Tree starter node."""

    def __init__(self, namespace: str, use_sim_time: bool = False) -> None:
        super().__init__('start_bt', namespace=namespace)

        self.param_use_sim_time = Parameter(
            'use_sim_time', Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([self.param_use_sim_time])

        self.start_pub = self.create_publisher(
            String, "start", qos_profile_system_default)

        self.start_pub.publish(String())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Single drone mission')

    parser.add_argument('-n', '--namespace',
                        type=str,
                        default='drone0',
                        help='ID of the drone to be used in the mission')
    parser.add_argument('-s', '--use_sim_time',
                        action='store_true',
                        default=False,
                        help='Use simulation time')

    args = parser.parse_args()
    drone_namespace = args.namespace
    use_sim_time = args.use_sim_time

    rclpy.init()
    start_bt = StartBehaviorTree(
        namespace=drone_namespace,
        use_sim_time=use_sim_time)
    rclpy.spin_once(start_bt, timeout_sec=1)
    rclpy.shutdown()
