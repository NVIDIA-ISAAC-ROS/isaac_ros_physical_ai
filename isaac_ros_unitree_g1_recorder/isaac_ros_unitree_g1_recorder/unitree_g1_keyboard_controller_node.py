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
Keyboard controller for the G1 recorder session manager.

Provides a curses-based terminal UI for controlling recording sessions.
Calls the recorder node's services on keypress.

Keys:
  Space  — Start (IDLE) / Stop & Save (RECORDING)
  c      — Cancel & discard (RECORDING only)
  t      — Change task description (IDLE only)
  q      — Quit (IDLE only)
"""

import curses
import json
import os
import sys
import threading
import time

from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


_DEFAULT_WORKSPACE_MOUNT = '/workspaces/isaac_ros-dev'


class UnitreeG1KeyboardControllerNode(Node):
    """Curses-based keyboard controller for the G1 recorder."""

    def __init__(self, **kwargs):
        super().__init__('g1_keyboard_controller', **kwargs)

        self.declare_parameter('recorder_node_name', '/g1_recorder')
        recorder = self.get_parameter('recorder_node_name').value

        # Service clients.
        self._start_client = self.create_client(
            Trigger, f'{recorder}/start_recording')
        self._stop_client = self.create_client(
            Trigger, f'{recorder}/stop_recording')
        self._cancel_client = self.create_client(
            Trigger, f'{recorder}/cancel_recording')
        self._set_params_client = self.create_client(
            SetParameters, f'{recorder}/set_parameters')

        # Status from recorder. Access is serialized via ``_status_lock`` —
        # ROS callbacks (spin thread) write, the curses loop reads.
        self._status_lock = threading.Lock()
        self._status = {
            'state': 'UNKNOWN',
            'task': '',
            'saved_episodes': 0,
            'current_episode_duration_s': 0.0,
            'current_episode_path': '',
        }
        self.create_subscription(
            String, f'{recorder}/status', self._on_status, 10)

    def _on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().debug(f'Invalid status JSON ({e}): {msg.data!r}')
            return
        with self._status_lock:
            self._status = data

    def _snapshot_status(self) -> dict:
        """Return a copy of the latest status for reading without holding the lock."""
        with self._status_lock:
            return dict(self._status)

    def _wait_for_future(self, future, timeout_sec: float = 10.0):
        """Poll a future until done. Safe to call while a background spin thread is running."""
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.result() if future.done() else None

    def _call_trigger(self, client) -> str:
        """Call a Trigger service. Returns the response message."""
        if not client.wait_for_service(timeout_sec=2.0):
            return 'Service not available'
        future = client.call_async(Trigger.Request())
        result = self._wait_for_future(future)
        if result is not None:
            return result.message if result.success else f'FAILED: {result.message}'
        return 'Service call timed out'

    def _set_task(self, new_task: str) -> str:
        """Set the task_description parameter on the recorder node."""
        if not self._set_params_client.wait_for_service(timeout_sec=2.0):
            return 'Parameter service not available'
        param = ParameterMsg()
        param.name = 'task_description'
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=new_task)
        req = SetParameters.Request()
        req.parameters = [param]
        future = self._set_params_client.call_async(req)
        result = self._wait_for_future(future, timeout_sec=5.0)
        if result is not None:
            results = result.results
            if results and results[0].successful:
                return f'Task set to: {new_task}'
            return f'Failed: {results[0].reason if results else "unknown"}'
        return 'Parameter service call timed out'

    @staticmethod
    def _to_host_path(container_path: str) -> str:
        """
        Convert a container path to the host path via mountinfo.

        The workspace mount point defaults to ``/workspaces/isaac_ros-dev``
        (Isaac ROS dev container convention) but can be overridden via the
        ``ISAAC_ROS_WS`` env var.

        mountinfo format: mount_id parent_id major:minor root mount_point ...
        For bind mounts, parts[3] (root) is the host source path and
        parts[4] (mount_point) is the container path.
        """
        ws = os.environ.get('ISAAC_ROS_WS', _DEFAULT_WORKSPACE_MOUNT)
        if not container_path.startswith(ws):
            return container_path
        try:
            with open('/proc/self/mountinfo') as f:
                for line in f:
                    parts = line.split()
                    if len(parts) >= 5 and parts[4] == ws:
                        return parts[3] + container_path[len(ws):]
        except OSError:
            pass
        return container_path

    def run_ui(self):
        """Run the curses UI. Blocks until the user quits."""
        if not sys.stdin.isatty() or not os.environ.get('TERM'):
            raise RuntimeError(
                'Keyboard controller requires an interactive terminal. '
                'Run directly in a TTY (not via ros2 launch) and make sure '
                "the TERM environment variable is set (e.g. 'xterm-256color')."
            )
        curses.wrapper(self._curses_main)

    def _curses_main(self, stdscr):
        stdscr.nodelay(True)
        curses.curs_set(0)
        curses.use_default_colors()

        # Spin ROS in a background thread.
        spin_thread = threading.Thread(
            target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()

        last_message = ''

        while True:
            # Exit if the recorder launch process died.
            launch_proc = getattr(self, 'launch_proc', None)
            if launch_proc and launch_proc.poll() is not None:
                curses.endwin()
                launch_log = getattr(self, 'launch_log_path', None)
                log_hint = f'Check {launch_log}' if launch_log else (
                    'Check the ros2 launch log.')
                print(f'Recorder process died. {log_hint}')
                break

            stdscr.clear()
            status = self._snapshot_status()
            state = status.get('state', 'UNKNOWN')
            task = status.get('task', '')
            saved = status.get('saved_episodes', 0)
            duration = status.get('current_episode_duration_s', 0.0)

            # Header.
            output_dir = status.get('output_dir', '')
            host_path = self._to_host_path(output_dir)
            stdscr.addstr(0, 0, f'Task: {task}')
            stdscr.addstr(1, 0, f'Saving to (host): {host_path}')
            status_line = f'Episode: {saved} saved | Status: {state}'
            if state == 'RECORDING':
                status_line += f'  {duration:.1f}s'
            elif state == 'INITIALIZING':
                status_line += '  (starting encoder...)'
            elif state == 'SAVING':
                status_line += '  (saving...)'
            elif state == 'CANCELING':
                status_line += '  (canceling...)'
            stdscr.addstr(2, 0, status_line)
            stdscr.addstr(3, 0, '─' * 40)

            # Controls.
            if state in ('INITIALIZING', 'SAVING', 'CANCELING'):
                stdscr.addstr(4, 0, 'Please wait...')
            elif state == 'RECORDING':
                stdscr.addstr(4, 0, '[Space] Stop & Save   [c] Cancel')
            else:
                stdscr.addstr(4, 0, '[Space] Start   [t] Change Task   [q] Quit')

            # Last message.
            if last_message:
                stdscr.addstr(6, 0, last_message[:60])

            stdscr.refresh()

            # Read key (non-blocking).
            try:
                key = stdscr.getch()
            except curses.error:
                key = -1

            if key == -1:
                curses.napms(100)
                continue

            if key == ord(' '):
                if state == 'IDLE' or state == 'UNKNOWN':
                    last_message = self._call_trigger(self._start_client)
                elif state == 'RECORDING':
                    last_message = self._call_trigger(self._stop_client)

            elif key == ord('c') and state in ('RECORDING', 'INITIALIZING'):
                last_message = self._call_trigger(self._cancel_client)

            elif key == ord('t') and state != 'RECORDING':
                # Drop out of curses for text input.
                curses.endwin()
                try:
                    try:
                        new_task = input('Enter new task description: ').strip()
                    except (EOFError, KeyboardInterrupt):
                        new_task = ''
                    if new_task:
                        last_message = self._set_task(new_task)
                    else:
                        last_message = 'Task unchanged'
                finally:
                    stdscr = curses.initscr()
                    curses.noecho()
                    curses.cbreak()
                    stdscr.keypad(True)
                    stdscr.nodelay(True)
                    curses.curs_set(0)

            elif key == ord('q') and state != 'RECORDING':
                break


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UnitreeG1KeyboardControllerNode()
        node.run_ui()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
