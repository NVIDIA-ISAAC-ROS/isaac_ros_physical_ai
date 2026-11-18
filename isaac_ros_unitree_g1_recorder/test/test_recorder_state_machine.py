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

"""Tests for the recorder node state machine and bag management."""

import json
from pathlib import Path
import shutil
import tempfile
import time
from unittest.mock import MagicMock

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def output_dir():
    d = tempfile.mkdtemp(prefix='test_recorder_')
    yield d
    shutil.rmtree(d, ignore_errors=True)


@pytest.fixture
def recorder_node(output_dir):
    from isaac_ros_unitree_g1_recorder.unitree_g1_recorder_node import (
        UnitreeG1RecorderNode,
    )

    node = UnitreeG1RecorderNode(
        parameter_overrides=[
            Parameter('task_description', value='test task'),
            Parameter('output_dir', value=output_dir),
            Parameter('sync_rate', value=15.0),
        ]
    )
    yield node
    node.destroy_node()


def _drive_cleanup(node):
    """Run the async cleanup pipeline to completion without spinning."""
    node._on_cleanup()
    if node._cleanup_thread is not None:
        node._cleanup_thread.join(timeout=10)
    node._on_cleanup_poll()


def _start_recording(node):
    """Start recording and force transition to RECORDING (skip async encoder)."""
    req = Trigger.Request()
    resp = Trigger.Response()
    node._on_start_recording(req, resp)
    assert resp.success is True
    # In tests, the encoder service isn't available, so the node stays
    # in INITIALIZING. Force it to RECORDING for state machine tests.
    assert node.state == 'INITIALIZING'
    node._state = 'RECORDING'
    node._recording_start_time = time.monotonic()
    return resp


def _stop_recording(node):
    """Stop recording and run async cleanup manually."""
    req = Trigger.Request()
    resp = Trigger.Response()
    node._on_stop_recording(req, resp)
    assert resp.success is True
    assert node.state == 'SAVING'
    _drive_cleanup(node)
    assert node.state == 'IDLE'
    return resp


def test_initial_state_is_idle(recorder_node):
    assert recorder_node.state == 'IDLE'


def test_start_recording_transitions_to_initializing(recorder_node):
    req = Trigger.Request()
    resp = Trigger.Response()
    recorder_node._on_start_recording(req, resp)
    assert resp.success is True
    assert recorder_node.state == 'INITIALIZING'


def test_start_recording_while_recording_fails(recorder_node):
    _start_recording(recorder_node)
    assert recorder_node.state == 'RECORDING'

    req = Trigger.Request()
    resp2 = Trigger.Response()
    recorder_node._on_start_recording(req, resp2)
    assert resp2.success is False
    assert 'RECORDING' in resp2.message


def test_stop_recording_transitions_to_idle(recorder_node):
    _start_recording(recorder_node)
    assert recorder_node.state == 'RECORDING'

    _stop_recording(recorder_node)
    assert recorder_node.saved_episode_count == 1


def test_stop_recording_while_idle_fails(recorder_node):
    req = Trigger.Request()
    resp = Trigger.Response()
    recorder_node._on_stop_recording(req, resp)
    assert resp.success is False


def test_cancel_recording_transitions_to_idle(recorder_node):
    _start_recording(recorder_node)
    episode_path = recorder_node._current_episode_path
    assert Path(episode_path).exists()

    req = Trigger.Request()
    resp2 = Trigger.Response()
    recorder_node._on_cancel_recording(req, resp2)
    assert resp2.success is True
    assert recorder_node.state == 'CANCELING'
    _drive_cleanup(recorder_node)
    assert recorder_node.state == 'IDLE'
    assert not Path(episode_path).exists()
    assert recorder_node.saved_episode_count == 0


def test_cancel_during_initializing(recorder_node):
    req = Trigger.Request()
    resp = Trigger.Response()
    recorder_node._on_start_recording(req, resp)
    assert recorder_node.state == 'INITIALIZING'
    episode_path = recorder_node._current_episode_path

    resp2 = Trigger.Response()
    recorder_node._on_cancel_recording(req, resp2)
    assert resp2.success is True
    assert recorder_node.state == 'CANCELING'
    _drive_cleanup(recorder_node)
    assert recorder_node.state == 'IDLE'
    assert not Path(episode_path).exists()


def test_cancel_recording_while_idle_fails(recorder_node):
    req = Trigger.Request()
    resp = Trigger.Response()
    recorder_node._on_cancel_recording(req, resp)
    assert resp.success is False


def test_episode_numbering_increments(recorder_node):
    for _ in range(3):
        _start_recording(recorder_node)
        _stop_recording(recorder_node)

    assert recorder_node.saved_episode_count == 3


def test_status_json_idle(recorder_node):
    status = json.loads(recorder_node._build_status_json())
    assert status['state'] == 'IDLE'
    assert status['task'] == 'test task'
    assert status['saved_episodes'] == 0
    assert status['current_episode_duration_s'] == 0.0
    assert status['current_episode_path'] == ''


def test_set_empty_task_description_rejected(recorder_node):
    """The parameter validation callback must reject empty task descriptions."""
    result = recorder_node.set_parameters(
        [Parameter('task_description', value='')]
    )
    assert result[0].successful is False
    # Non-empty update is accepted.
    result = recorder_node.set_parameters(
        [Parameter('task_description', value='pick up the red cup')]
    )
    assert result[0].successful is True
    assert (recorder_node.get_parameter('task_description').value
            == 'pick up the red cup')


def test_start_recording_popen_failure_cleans_up(recorder_node, monkeypatch):
    """If ros2 bag fails to launch, the log fd and episode dir are cleaned up."""
    import subprocess as _subprocess

    def _raise_oserror(*_args, **_kwargs):
        raise OSError('ros2 not found')

    monkeypatch.setattr(_subprocess, 'Popen', _raise_oserror)

    req = Trigger.Request()
    resp = Trigger.Response()
    recorder_node._on_start_recording(req, resp)

    assert resp.success is False
    assert recorder_node.state == 'IDLE'
    assert recorder_node._bag_log_file is None
    assert recorder_node._bag_process is None
    # Episode directory must have been removed.
    for episode_dir in Path(recorder_node._output_dir).glob('episode_*'):
        pytest.fail(f'Leftover episode dir: {episode_dir}')


def test_finalize_cleanup_handles_none_start_time(recorder_node):
    """_finalize_cleanup must not crash if _recording_start_time is None."""
    recorder_node._current_episode_path = str(
        Path(recorder_node._output_dir) / 'fake_ep')
    recorder_node._recording_start_time = None  # simulate abort before RECORDING
    # Should not raise.
    recorder_node._finalize_cleanup('save')
    assert recorder_node.state == 'IDLE'


def _start_and_swap_bag_process(node, mock_poll_return):
    """
    Start recording (real subprocess), then reap it and substitute a mock.

    Several init-path tests need deterministic control over bag_process.poll()
    without leaving the real ``ros2 bag record`` process running.
    """
    req = Trigger.Request()
    resp = Trigger.Response()
    node._on_start_recording(req, resp)
    assert resp.success
    node._stop_bag_process()  # reap the real subprocess before swapping
    fake_proc = MagicMock()
    fake_proc.poll.return_value = mock_poll_return
    node._bag_process = fake_proc
    return Path(node._current_episode_path)


def test_encoder_load_failure_aborts_episode(recorder_node):
    """Failed encoder load during init routes episode through async cancel."""
    episode_path = _start_and_swap_bag_process(
        recorder_node, mock_poll_return=None)

    future = MagicMock()
    future.done.return_value = True
    future.result.return_value = MagicMock(
        success=False, error_message='boom', unique_id=0)
    recorder_node._encoder_load_future = future

    recorder_node._on_init_poll()

    # Phase 2 failures dispatch cleanup to the async worker rather than
    # blocking the executor on proc.wait; state advances to CANCELING and
    # the next cleanup timer tick + async worker finalize it to IDLE.
    assert recorder_node.state == 'CANCELING'
    assert recorder_node._cleanup_action == 'cancel'
    assert not recorder_node._cleanup_timer.is_canceled()
    # Episode directory still exists until the async cleanup tears it down.
    assert episode_path.exists()


def test_bag_death_during_init_aborts_episode(recorder_node):
    """If the bag process dies during INITIALIZING, init aborts cleanly."""
    episode_path = _start_and_swap_bag_process(
        recorder_node, mock_poll_return=1)  # already exited
    recorder_node._bag_ready_time = 0.0

    recorder_node._on_init_poll()

    assert recorder_node.state == 'IDLE'
    assert not episode_path.exists()


def test_load_encoder_issues_service_request(recorder_node):
    """_load_encoder populates a future via the LoadNode client."""
    fake_future = object()
    recorder_node._load_client.call_async = MagicMock(return_value=fake_future)

    recorder_node._load_encoder()

    assert recorder_node._encoder_load_future is fake_future
    recorder_node._load_client.call_async.assert_called_once()
    req = recorder_node._load_client.call_async.call_args[0][0]
    assert req.package_name == 'isaac_ros_h264_encoder'
    assert req.plugin_name.endswith('::EncoderNode')


def test_init_poll_loads_encoder_after_bag_ready(recorder_node):
    """Phase 1 of _on_init_poll creates publishers and issues the load."""
    _start_and_swap_bag_process(recorder_node, mock_poll_return=None)
    recorder_node._bag_ready_time = 0.0
    recorder_node._load_client.call_async = MagicMock(return_value=MagicMock())

    recorder_node._on_init_poll()

    assert recorder_node._encoder_load_future is not None
    recorder_node._load_client.call_async.assert_called_once()
    recorder_node._bag_process = None
