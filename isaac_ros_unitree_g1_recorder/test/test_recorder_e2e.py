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
Recorder test with fake input data (no MuJoCo, no encoder).

Instantiates ``UnitreeG1RecorderNode`` in-process, publishes fake data on
every topic the recorder subscribes to, drives the recorder through its
state machine bypassing the encoder composition service (which is not
available in this test context), and verifies the resulting MCAP bag
contains messages on all expected topics.

End-to-end coverage that includes the real MuJoCo + IK + finger stack
lives in ``test_recorder_with_mujoco.launch.py``.

Requires a ROS 2 environment (runs inside the container).
"""

import logging
from pathlib import Path
import shutil
import tempfile
import threading
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
import pytest
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Imu, JointState
from std_msgs.msg import Header
from std_srvs.srv import Trigger


@pytest.fixture(scope='module')
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def output_dir():
    d = tempfile.mkdtemp(prefix='test_e2e_recorder_')
    yield d
    shutil.rmtree(d, ignore_errors=True)


def _make_header(node: Node) -> Header:
    now = node.get_clock().now().to_msg()
    return Header(stamp=now)


def _make_joint_state(node: Node, n_joints: int = 43) -> JointState:
    msg = JointState()
    msg.header = _make_header(node)
    msg.name = [f'joint_{i}' for i in range(n_joints)]
    msg.position = [0.0] * n_joints
    msg.velocity = [0.0] * n_joints
    msg.effort = [0.0] * n_joints
    return msg


def _make_imu(node: Node) -> Imu:
    msg = Imu()
    msg.header = _make_header(node)
    return msg


def _make_compressed_image(node: Node) -> CompressedImage:
    msg = CompressedImage()
    msg.header = _make_header(node)
    msg.format = 'h264'
    msg.data = bytes(100)  # dummy payload
    return msg


def _make_twist_stamped(node: Node) -> TwistStamped:
    msg = TwistStamped()
    msg.header = _make_header(node)
    return msg


def _make_pose_stamped(node: Node) -> PoseStamped:
    msg = PoseStamped()
    msg.header = _make_header(node)
    return msg


class FakePublisherNode(Node):
    """Publishes fake messages on all topics the recorder expects."""

    def __init__(self):
        super().__init__('fake_publisher')
        qos = qos_profile_sensor_data
        self.js_pub = self.create_publisher(JointState, '/joint_states', qos)
        self.imu_pub = self.create_publisher(Imu, '/imu_sensor_broadcaster/imu', qos)
        self.cam_pub = self.create_publisher(
            CompressedImage, '/camera/color/image_compressed', qos)
        self.twist_pub = self.create_publisher(
            TwistStamped, '/xr_teleop/root_twist', qos)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/xr_teleop/root_pose', qos)

    def publish_all(self):
        self.js_pub.publish(_make_joint_state(self))
        self.imu_pub.publish(_make_imu(self))
        self.cam_pub.publish(_make_compressed_image(self))
        self.twist_pub.publish(_make_twist_stamped(self))
        self.pose_pub.publish(_make_pose_stamped(self))


def test_record_episode_captures_data(ros_context, output_dir):
    """Record a short episode with fake data and verify the bag."""
    from isaac_ros_unitree_g1_recorder.unitree_g1_recorder_node import (
        UnitreeG1RecorderNode,
    )

    recorder = UnitreeG1RecorderNode(
        parameter_overrides=[
            Parameter('task_description', value='e2e test'),
            Parameter('output_dir', value=output_dir),
            Parameter('sync_rate', value=10.0),
        ]
    )
    publisher = FakePublisherNode()

    # Use the same executor type as production (SingleThreadedExecutor
    # via rclpy.spin). We need a background thread so the test can
    # control timing, but both nodes share one executor thread.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(recorder)
    executor.add_node(publisher)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Publish for 1s so stamps are populated.
        for _ in range(10):
            publisher.publish_all()
            time.sleep(0.1)

        # Start recording. The encoder composition service is not
        # available in tests, so the node stays in INITIALIZING. Drive
        # the state machine to RECORDING manually so we exercise the
        # data-recording path without the encoder. End-to-end encoder
        # coverage lives in test_recorder_with_mujoco.launch.py.
        req = Trigger.Request()
        resp = Trigger.Response()
        recorder._on_start_recording(req, resp)
        assert resp.success
        assert recorder.state == 'INITIALIZING'

        # Give ros2 bag record time to subscribe to the requested topics
        # before we create the per-episode publishers — the bag needs to
        # see each publisher's DDS announcement to pick up the message
        # type and record anything.
        time.sleep(2.0)

        # Production code creates these publishers during INITIALIZING
        # after the bag is ready; replicate that here since we bypassed
        # the encoder load path that would normally trigger it.
        recorder._create_episode_publishers()

        recorder._state = 'RECORDING'
        recorder._recording_start_time = time.monotonic()

        # Publish during recording.
        for _ in range(30):
            publisher.publish_all()
            time.sleep(0.1)

        # Stop recording (async — returns SAVING, cleanup in timer).
        resp2 = Trigger.Response()
        recorder._on_stop_recording(req, resp2)
        assert resp2.success
        # Wait for async cleanup to complete.
        deadline = time.monotonic() + 10
        while recorder.state != 'IDLE' and time.monotonic() < deadline:
            time.sleep(0.1)
        assert recorder.state == 'IDLE'
        assert recorder.saved_episode_count == 1

        # Find the episode directory.
        session_dirs = list(Path(output_dir).glob('session_*'))
        assert len(session_dirs) == 1
        episode_dirs = list(session_dirs[0].glob('episode_*'))
        assert len(episode_dirs) == 1

        episode = episode_dirs[0]
        assert (episode / 'task.txt').exists()
        assert (episode / 'bag').exists()

        # Verify bag contents using rosbags.
        bag_path = episode / 'bag'
        try:
            from rosbags.rosbag2 import Reader
            with Reader(bag_path) as reader:
                topics_with_data = {}
                for conn, _, _ in reader.messages():
                    topics_with_data[conn.topic] = (
                        topics_with_data.get(conn.topic, 0) + 1
                    )

            expected_topics = [
                '/joint_states',
                '/camera/color/image_compressed',
                '/imu_sensor_broadcaster/imu',
                '/xr_teleop/root_twist',
                '/xr_teleop/root_pose',
                '/recording/task_description',
            ]
            for topic in expected_topics:
                assert topic in topics_with_data, (
                    f'{topic} missing from bag. Got: {list(topics_with_data)}'
                )
                assert topics_with_data[topic] > 0, (
                    f'{topic} has 0 messages'
                )

            logging.info('Bag contents: %s', topics_with_data)

        except ImportError:
            # rosbags not installed — just check bag directory exists.
            mcap_files = list(bag_path.glob('*.mcap'))
            assert len(mcap_files) > 0, 'No MCAP files in bag directory'

    finally:
        recorder._stop_bag_process()
        executor.shutdown(timeout_sec=2)
        recorder.destroy_node()
        publisher.destroy_node()
