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

"""Launch tmuxinator commands in separate gnome terminals."""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import subprocess
from pathlib import Path


def launch_terminals(cmd_lists: list):
    """Launch gnome terminals with the given commands.

    :param cmd_lists: List of commands to execute in each terminal
    :type cmd_lists: list
    """
    # Add gnome tab to each command
    for i, cmd in enumerate(cmd_lists):
        cmd_lists[i] = f'gnome-terminal --tab -- bash -c "sleep 1s; {cmd}; exec bash -i";'
    
    # Join the commands into a single string with spaces
    cmd = ' '.join(cmd_lists)

    # Execute the command using subprocess
    cmd = f"gnome-terminal --window -- bash -c '{cmd}'"

    subprocess.run(cmd, shell=True, check=True)


def tmuxinator_parse(tmunixator_ouput: str) -> list:
    """Parse tmuxinator debug output to extract the commands.

    :param tmunixator_ouput: tmuxinator debug output
    :type tmunixator_ouput: str
    :return: List of commands to execute in each terminal
    :rtype: list
    """
    import re
    # Extract the initial directory
    initial_cd_cmd = ''
    for line in tmunixator_ouput:
        if line.startswith("cd "):
            initial_cd_cmd = line.strip()
            initial_dir = line.strip().split(" ")[1]
            break
    
    # Extract tmux send-keys commands
    send_keys_pattern = re.compile(r"tmux send-keys -t \S+ (.+) C-m")
    commands = []
    for line in tmunixator_ouput:
        match = send_keys_pattern.search(line)
        if match:
            command = match.group(1).replace('\\', '')
            commands.append(f'{initial_cd_cmd}; echo ${command}')
    return commands

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Launch tmuxinator commands in separate gnome terminals.')
    parser.add_argument('-p', '--path', type=str, required=True,
                        help='tmuxinator debug output file path.')
    args = parser.parse_args()

    file_path = Path(args.path)
    if not file_path.exists():
        raise FileNotFoundError(f"File {file_path} not found")

    with open(file_path, 'r') as file:
        tmuxinator_debug_output = file.readlines()
    commands = tmuxinator_parse(tmuxinator_debug_output)
    launch_terminals(commands)
