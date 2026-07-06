#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Single-terminal entry point: launches the recorder backend and keyboard UI.

Usage:
  ros2 run isaac_ros_unitree_g1_recorder record -- task_description:='pick up the red cup'

All arguments after '--' are forwarded to the launch file.
"""

from pathlib import Path
import signal
import subprocess
import sys
import tempfile

from isaac_ros_unitree_g1_recorder.unitree_g1_keyboard_controller_node import (
    UnitreeG1KeyboardControllerNode,
)
import rclpy


def _print_session_summary(output_dir: str, saved_episodes: int):
    """Print session summary on quit."""
    if saved_episodes == 0:
        return

    host_path = UnitreeG1KeyboardControllerNode._to_host_path(output_dir)
    print(f'\n{saved_episodes} episode(s) saved to: {host_path}')


def main():
    # Forward all args to the launch file.  ``ros2 run <pkg> record -- <args>``
    # passes a literal ``--`` separator through in argv; strip any leading
    # occurrences so it doesn't end up in the ros2 launch command line.
    launch_args = sys.argv[1:]
    while launch_args and launch_args[0] == '--':
        launch_args = launch_args[1:]
    launch_cmd = [
        'ros2', 'launch',
        'isaac_ros_unitree_g1_recorder', 'unitree_g1_recorder.launch.py',
    ] + launch_args

    launch_log = Path(tempfile.gettempdir()) / 'g1_recorder_launch.log'
    with launch_log.open('w') as launch_log_file:
        launch_proc = subprocess.Popen(
            launch_cmd,
            stdout=launch_log_file,
            stderr=subprocess.STDOUT,
        )

        node = None
        status = {}
        try:
            rclpy.init()
            node = UnitreeG1KeyboardControllerNode()
            node.launch_proc = launch_proc
            node.launch_log_path = str(launch_log)
            node.run_ui()
            status = node._snapshot_status()
        except KeyboardInterrupt:
            pass
        finally:
            if node:
                node.destroy_node()
            rclpy.try_shutdown()
            if launch_proc.poll() is None:
                launch_proc.send_signal(signal.SIGINT)
                try:
                    launch_proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    launch_proc.kill()
                    launch_proc.wait()

    _print_session_summary(
        status.get('output_dir', ''),
        status.get('saved_episodes', 0),
    )


if __name__ == '__main__':
    main()
