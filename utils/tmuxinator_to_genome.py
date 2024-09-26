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
from pathlib import Path
import re
import subprocess


class GnomeTerminal:
    def __init__(self):
        self.commands = []
        self.initial_dir = None

    def parse_tmuxinator_debug(self, filepath):
        """Parse the tmuxinator debug output file to extract tmux send-keys commands."""
        print(f'Parsing the file: {filepath}')
        with open(filepath, 'r') as file:
            lines = file.readlines()

        # Extract the initial directory
        for line in lines:
            if line.startswith('cd '):
                self.initial_dir = line.strip().split(' ')[1]
                break

        # Extract tmux send-keys commands
        send_keys_pattern = re.compile(r'tmux send-keys -t \S+ (.+) C-m')
        for line in lines:
            match = send_keys_pattern.search(line)
            if match:
                command = match.group(1).replace('\\', '')
                self.commands.append(command)
                print(f'Command extracted: {command}')

    def open_gnome_terminal_with_tabs(self):
        """Open GNOME Terminal with tabs and execute commands in each tab."""
        if not self.commands or not self.initial_dir:
            print('No commands or initial directory found.')
            return

        global_cmd = []
        for cmd in self.commands:
            global_cmd.append(
                f'gnome-terminal --tab -- bash -c "sleep 0.1s; {cmd}; exec bash -i";')

        cmd = ' '.join(global_cmd)
        launch_command = f"gnome-terminal --window -- bash -c '{cmd}'"
        # Execute the final command
        subprocess.run(launch_command, shell=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Launch tmuxinator commands in separate gnome terminals.')
    parser.add_argument('-p', '--path', type=str, required=True,
                        help='tmuxinator debug output file path.')
    args = parser.parse_args()

    file_path = Path(args.path)
    if not file_path.exists():
        raise FileNotFoundError(f'File {file_path} not found')

    gnome_terminal = GnomeTerminal()
    gnome_terminal.parse_tmuxinator_debug(file_path)
    gnome_terminal.open_gnome_terminal_with_tabs()
