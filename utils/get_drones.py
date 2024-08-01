# /usr/bin/env python3

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

"""Get drones names from config file"""

__authors__ = 'Rafael Perez-Segui, Pedro Arias-Perez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import yaml
from pathlib import Path
import json


def read_file(filename: Path) -> str:
    """Read file content

    :param filename: Path to file
    :type filename: Path
    :return: File content
    :rtype: str
    """
    filename = str(filename)
    # Check extension of config file
    if filename.endswith('.json'):
        with open(filename, 'r', encoding='utf-8') as stream:
            config = json.load(stream)
    elif filename.endswith('.yaml') or filename.endswith('.yml'):
        with open(filename, 'r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
    else:
        raise ValueError('Invalid configuration file extension.')
    return config


def get_drones_namespaces(filename: Path) -> list[str]:
    """Get drone namespaces listed in config file (JSON or YAML)
    Open file, read as JSON or YAML depending on file extension, and return namespaces as list

    :param filename: Path to drones config file
    :type filename: Path
    :return: List of drone namespaces
    :rtype: list[str]
    """
    config = read_file(filename)
    drones_namespaces=[]
    
    # For Gazebo and PX4 SITL
    if 'drones' in config:
        for drone in config['drones']:
            # Gazebo
            if 'model_name' in drone:
                drones_namespaces.append(drone["model_name"])
            # PX4 SITL
            elif 'namespace' in drone:
                drones_namespaces.append(drone["namespace"])
    # AS2 Multirotor Simulator
    else:
        for drone in config:
            if drone == '/**':
                continue
            drones_namespaces.append(drone)

    if len(drones_namespaces) == 0:
        raise ValueError("No drones found in config file")
    return drones_namespaces


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--config_file', type=str, required=True, help="Path to drones config file")
    parser.add_argument('-s', '--separator', type=str, help="Separator", default=":")
    args = parser.parse_args()

    config_file = Path(args.config_file)
    if not config_file.exists():
        raise FileNotFoundError(f"File {config_file} not found")

    # Get drones from config file
    drones = get_drones_namespaces(config_file)

    # Return for use in bash scripts
    print(args.separator.join(drones))
