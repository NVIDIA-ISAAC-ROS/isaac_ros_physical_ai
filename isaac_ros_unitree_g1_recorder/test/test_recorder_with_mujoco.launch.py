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
End-to-end launch test for the G1 recorder driven by MuJoCo teleop stack.

Launches the full G1 controller-manager stack in MuJoCo with the
``agile_velocity_with_ik`` controller group, alongside the recorder and
its H264 encoder container. Mock teleop commands are published directly
from the test (right-hand end-effector pose and closed left fingers) to
replace the real XR teleop app.  After a short recording session, the
resulting MCAP bag is opened and asserted to contain messages on every
topic the recorder is configured to capture.

Smaller, encoder-free coverage lives in ``test_recorder_e2e.py``.
"""

import json
import os
from pathlib import Path
import shutil
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
import launch
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from mujoco_test_helpers import (
    spin_for,
    wait_for_controllers,
)
import rclpy
import rclpy.parameter
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


# Scripted two-phase teleop for an intuitive demo:
#   Phase 1 (rest): both hands out in front at ~belly-button height, palms
#                   facing forward, left fingers open.
#   Phase 2 (lift): right hand lifts ~25 cm, left hand stays put, left
#                   fingers close.
# Poses are in world_teleop (== pelvis via the static identity TF below).
# Identity orientation sends the palm-link fingers along pelvis -z (down),
# so we rotate -90° around pelvis +y (pitch) to swing them to pelvis +x
# (forward).  Both hands share this orientation — the lateral offset (±y)
# puts them on each side of the body.
def _pose(x: float, y: float, z: float,
          qx: float = 0.0, qy: float = 0.0,
          qz: float = 0.0, qw: float = 1.0) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p


# -90° about pelvis +y ≈ (qx=0, qy=-sin(45°), qz=0, qw=cos(45°)).
_FORWARD_QY = -0.7071068
_FORWARD_QW = 0.7071068

_REST_RIGHT_POSE = _pose(x=0.30, y=-0.18, z=0.0,
                         qy=_FORWARD_QY, qw=_FORWARD_QW)
_REST_LEFT_POSE = _pose(x=0.30, y=0.18, z=0.0,
                        qy=_FORWARD_QY, qw=_FORWARD_QW)
_LIFT_RIGHT_POSE = _pose(x=0.30, y=-0.18, z=0.25,
                         qy=_FORWARD_QY, qw=_FORWARD_QW)

# Finger joint positions for each phase.  Each joint has its own valid
# range in the URDF — notably the index/middle joints close toward negative
# values (range ~[-1.8, 0.2]), while most thumb joints close toward positive
# values.  ``thumb_0`` (rotation axis) is held at 0 so only curl actuates.
# Right-hand fingers are always held at 0 (not commanded open/close in
# this test).  Joint names must match the finger controller's list in
# ``unitree_g1_bringup/config/controller_manager.yaml`` (post-rename, as
# seen on ``/xr_teleop/finger_joints``).
_FINGER_POSITIONS = {
    # Left hand — closes during Phase 2.
    # thumb: thumb_0 (rotation) stays at 0; thumb_1/2 close to positive
    'left_hand_thumb_0_joint':   {'open': 0.0, 'closed': 0.0},
    'left_hand_thumb_1_joint':   {'open': 0.0, 'closed': 0.7},
    'left_hand_thumb_2_joint':   {'open': 0.0, 'closed': 1.5},
    # index + middle: close toward negative
    'left_hand_index_0_joint':   {'open': 0.0, 'closed': -1.5},
    'left_hand_index_1_joint':   {'open': 0.0, 'closed': -1.5},
    'left_hand_middle_0_joint':  {'open': 0.0, 'closed': -1.5},
    'left_hand_middle_1_joint':  {'open': 0.0, 'closed': -1.5},
    # Right hand — held at 0 throughout.
    'right_hand_thumb_0_joint':  {'open': 0.0, 'closed': 0.0},
    'right_hand_thumb_1_joint':  {'open': 0.0, 'closed': 0.0},
    'right_hand_thumb_2_joint':  {'open': 0.0, 'closed': 0.0},
    'right_hand_index_0_joint':  {'open': 0.0, 'closed': 0.0},
    'right_hand_index_1_joint':  {'open': 0.0, 'closed': 0.0},
    'right_hand_middle_0_joint': {'open': 0.0, 'closed': 0.0},
    'right_hand_middle_1_joint': {'open': 0.0, 'closed': 0.0},
}
_FINGER_JOINTS = list(_FINGER_POSITIONS)

_PHASE_REST = 'rest'
_PHASE_LIFT = 'lift'

_TASK_DESCRIPTION = 'grasp with right hand and close left fingers'
_OUTPUT_BASE_DIR = Path(
    os.environ.get(
        'TEST_RECORDER_WITH_MUJOCO_OUTPUT_DIR',
        tempfile.mkdtemp(prefix='test_recorder_with_mujoco_'),
    )
)
# Leave the output tree in place if the operator set an output dir
# explicitly (for post-run bag inspection); otherwise clean up.
_CLEANUP_OUTPUT_DIR = 'TEST_RECORDER_WITH_MUJOCO_OUTPUT_DIR' not in os.environ

# Expected publication rates (Hz) per topic as ``(target, min_acceptable)``.
# The recorder runs under sim time (use_sim_time=true) and passes
# --use-sim-time to ros2 bag record, so bag log timestamps equal message
# header stamps in sim time — rates are invariant to how fast MuJoCo
# actually simulates on the host.  Bounds sit at ~75% of target to
# tolerate CI jitter.  Camera and /record_data come either from MuJoCo
# (host display available) or from the test's mock image stream
# (headless); both paths produce ~30 Hz.
_EXPECTED_RATES_HZ = {
    '/joint_states': (200.0, 150.0),
    '/applied_joint_commands': (200.0, 150.0),
    '/imu_sensor_broadcaster/imu': (200.0, 150.0),
    '/camera/color/image_compressed': (30.0, 22.0),
    # CameraInfo is published alongside each frame by MuJoCo / RealSense.
    '/realsense_d435_rgb/color/camera_info': (30.0, 22.0),
    '/record_data': (30.0, 22.0),
    # Raw XR teleop intent (pre-IK / pre-finger-controller) at the test's
    # own mock-publisher rate.
    '/xr_teleop/ee_poses': (20.0, 15.0),
    '/xr_teleop/finger_joints': (20.0, 15.0),
    '/xr_teleop/root_twist': (20.0, 15.0),
    '/xr_teleop/root_pose': (20.0, 15.0),
    # robot_state_publisher publishes /tf in lockstep with /joint_states.
    '/tf': (200.0, 150.0),
}
# TRANSIENT_LOCAL topics — only assert >=1 message (latched, published once).
_LATCHED_TOPICS = [
    '/recording/task_description',
    '/robot_description',
    '/tf_static',
]


def _static_tf(parent, child, tx=0.0, ty=0.0, tz=0.0,
               qx='0', qy='0', qz='0', qw='1'):
    """Return a static_transform_publisher Node for the given parent→child edge."""
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{parent}_to_{child}_tf'.replace('/', '_'),
        arguments=[
            '--frame-id', parent,
            '--child-frame-id', child,
            '--x', str(tx), '--y', str(ty), '--z', str(tz),
            '--qx', qx, '--qy', qy, '--qz', qz, '--qw', qw,
        ],
        output='screen',
    )


# Host X server socket + cookie as mounted into the dev container.  When
# both are present the admin user in this container can reach the host's
# NVIDIA-backed X server, so MuJoCo's GLFW camera renders at 30 Hz.  When
# either is missing (e.g. CI), we fall back to a mock raw-camera stream
# driven from the test itself.  TODO: drop the mock once mujoco_ros2_
# control gains an EGL backend for truly headless GPU rendering.
_HOST_DISPLAY = ':1'
_HOST_X_SOCKET = f'/tmp/.X11-unix/X{_HOST_DISPLAY[1:]}'
_HOST_XAUTHORITY = '/home/admin/.Xauthority'


def _host_display_available() -> bool:
    """Whether MuJoCo's GLFW camera can connect to the host X server.

    Needs the X socket, the admin user's Xauthority cookie, and that we
    run as the user who owns that cookie.  The host X server (Xwayland)
    does peer-credential UID checking on its unix socket and rejects any
    process whose euid doesn't match the session owner, even when the
    MIT-MAGIC-COOKIE is valid — typical ``colcon test`` / CI case.
    """
    if not Path(_HOST_X_SOCKET).exists():
        return False
    try:
        cookie_uid = os.stat(_HOST_XAUTHORITY).st_uid
    except (FileNotFoundError, PermissionError):
        return False
    return cookie_uid == os.geteuid()


def generate_test_description():
    """Launch MuJoCo + G1 controllers + recorder + encoder container."""
    bringup_share = Path(get_package_share_directory('unitree_g1_bringup'))
    recorder_share = Path(
        get_package_share_directory('isaac_ros_unitree_g1_recorder')
    )

    if _host_display_available():
        display_setup = [
            SetEnvironmentVariable('DISPLAY', _HOST_DISPLAY),
            SetEnvironmentVariable('XAUTHORITY', _HOST_XAUTHORITY),
        ]
    else:
        display_setup = []

    controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / 'launch/unitree_g1_controller_manager.launch.py')
        ),
        launch_arguments={
            'initial_controller_group': 'agile_velocity_with_ik',
            # Headless hosts (CI, root-launched colcon test): no DISPLAY
            # → MuJoCo's GLFW camera fails and TestRecorderWithMuJoCo
            # feeds a mock raw image stream instead.  When a host X
            # server is available, MuJoCo's own camera takes over.
            'enable_viewer': 'false',
            'use_foxglove': 'false',
            'use_rviz': 'false',
            # Drive IK from the mock xr_teleop topic the test publishes on.
            'ik_reference_pose_topic': '/xr_teleop/ee_poses',
            'cmd_vel_topic': '/xr_teleop/root_twist',
        }.items(),
    )

    # Static TF bridges between ROS and OpenXR conventions. Copied from the
    # production unitree_g1_teleop.launch.py so the IK controller resolves
    # its ``ee_command_frame → ee_frame`` lookups without needing the real
    # teleop app.
    _openxr_R_ros = ('-0.5', '0.5', '0.5', '0.5')
    _ros_R_openxr = ('0.5', '-0.5', '-0.5', '0.5')
    static_tfs = [
        # world_teleop->pelvis: owned by pose_reset_node in the teleop stack; a static
        # identity here so world_teleop-framed ee_poses resolve to pelvis.
        _static_tf('world_teleop', 'pelvis'),
        _static_tf('pelvis', 'world_openxr', tz=-1.0,
                   qx=_ros_R_openxr[0], qy=_ros_R_openxr[1],
                   qz=_ros_R_openxr[2], qw=_ros_R_openxr[3]),
        _static_tf('left_wrist_openxr', 'left_wrist',
                   qx=_openxr_R_ros[0], qy=_openxr_R_ros[1],
                   qz=_openxr_R_ros[2], qw=_openxr_R_ros[3]),
        _static_tf('right_wrist_openxr', 'right_wrist',
                   qx=_openxr_R_ros[0], qy=_openxr_R_ros[1],
                   qz=_openxr_R_ros[2], qw=_openxr_R_ros[3]),
        _static_tf('left_hand_palm_link', 'left_hand_palm_link_openxr',
                   qx=_ros_R_openxr[0], qy=_ros_R_openxr[1],
                   qz=_ros_R_openxr[2], qw=_ros_R_openxr[3]),
        _static_tf('right_hand_palm_link', 'right_hand_palm_link_openxr',
                   qx=_ros_R_openxr[0], qy=_ros_R_openxr[1],
                   qz=_ros_R_openxr[2], qw=_ros_R_openxr[3]),
    ]

    recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(recorder_share / 'launch/unitree_g1_recorder.launch.py')
        ),
        launch_arguments={
            'task_description': _TASK_DESCRIPTION,
            'output_dir': str(_OUTPUT_BASE_DIR),
            'sync_rate': '30.0',
            # MuJoCo publishes /clock; use it everywhere (recorder + bag
            # subprocess) so bag log_time matches header stamps and the
            # observed rates are invariant to host-side sim slowdown.
            'use_sim_time': 'true',
        }.items(),
    )

    return launch.LaunchDescription([
        *display_setup,
        controller_manager,
        *static_tfs,
        recorder_launch,
        launch_testing.actions.ReadyToTest(),
    ])


class TestRecorderWithMuJoCo(unittest.TestCase):
    """Records a short episode with mock teleop and validates the bag."""

    CONTROLLERS = [
        'inference_controller',
        'safety_controller',
        'ik_controller',
        'fingers_forward_joint_command_controller',
        # Spawned in a separate spawner chained after safety_controller; wait
        # for it too so recording doesn't start before /applied_joint_commands
        # flows.
        'joint_command_broadcaster',
    ]
    CONTROLLER_STARTUP_WAIT_S = 120.0
    STABILIZATION_WAIT_S = 2.0
    # Recording is split into two phases for a visually-meaningful demo
    # (see _publish_mock_teleop): rest → lift-right-hand + close-left-fingers.
    REST_PHASE_DURATION_S = 2.0
    LIFT_PHASE_DURATION_S = 3.0
    RECORDING_DURATION_S = REST_PHASE_DURATION_S + LIFT_PHASE_DURATION_S
    RECORDER_STATE_TIMEOUT_S = 30.0

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(
            'test_recorder_with_mujoco_node',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', value=True),
            ],
        )

        cls._status_messages: list[str] = []
        cls.node.create_subscription(
            String,
            '/g1_recorder/status',
            lambda msg: cls._status_messages.append(msg.data),
            10,
        )

        cls.ee_pub = cls.node.create_publisher(
            PoseArray, '/xr_teleop/ee_poses', 10)
        cls.finger_pub = cls.node.create_publisher(
            JointState, '/xr_teleop/finger_joints', 10)
        cls.twist_pub = cls.node.create_publisher(
            TwistStamped, '/xr_teleop/root_twist', 10)
        cls.pose_pub = cls.node.create_publisher(
            PoseStamped, '/xr_teleop/root_pose', 10)

        cls.start_cli = cls.node.create_client(
            Trigger, '/g1_recorder/start_recording')
        cls.stop_cli = cls.node.create_client(
            Trigger, '/g1_recorder/stop_recording')

        # Start in REST so hands settle in front of the robot before the
        # test flips to LIFT mid-recording.
        cls._phase = _PHASE_REST
        cls._teleop_timer = cls.node.create_timer(
            1.0 / 20.0, cls._publish_mock_teleop)

        # When no host X server is reachable MuJoCo's GLFW-based camera
        # renderer is disabled; publish a black 640×480 RGB8 raw frame
        # and a matching CameraInfo so the H264 encoder + recorder
        # pipeline still has input.  When a host display IS available
        # MuJoCo publishes its own camera and we skip the mock.
        cls._camera_timer = None
        if not _host_display_available():
            cls.camera_pub = cls.node.create_publisher(
                Image, '/realsense_d435_rgb/color/image_raw', 10)
            cls.camera_info_pub = cls.node.create_publisher(
                CameraInfo,
                '/realsense_d435_rgb/color/camera_info', 10)
            cls._camera_frame = bytes(640 * 480 * 3)
            cls._camera_timer = cls.node.create_timer(
                1.0 / 30.0, cls._publish_mock_camera)

    @classmethod
    def tearDownClass(cls):
        cls._teleop_timer.cancel()
        if cls._camera_timer is not None:
            cls._camera_timer.cancel()
        cls.node.destroy_node()
        rclpy.shutdown()
        if _CLEANUP_OUTPUT_DIR:
            shutil.rmtree(_OUTPUT_BASE_DIR, ignore_errors=True)

    @classmethod
    def _publish_mock_teleop(cls):
        """Publish one tick of mock teleop data at 20 Hz (phase-dependent)."""
        now = cls.node.get_clock().now().to_msg()

        if cls._phase == _PHASE_LIFT:
            right_pose = _LIFT_RIGHT_POSE
            left_pose = _REST_LEFT_POSE
            finger_key = 'closed'
        else:
            right_pose = _REST_RIGHT_POSE
            left_pose = _REST_LEFT_POSE
            finger_key = 'open'

        # ee_poses ordering convention: poses[0]=left, poses[1]=right (per BimanualIkController).
        ee = PoseArray()
        ee.header.stamp = now
        ee.header.frame_id = 'world_teleop'
        ee.poses.append(left_pose)
        ee.poses.append(right_pose)
        cls.ee_pub.publish(ee)

        fingers = JointState()
        fingers.header.stamp = now
        fingers.name = _FINGER_JOINTS
        fingers.position = [
            _FINGER_POSITIONS[j][finger_key] for j in _FINGER_JOINTS
        ]
        fingers.velocity = [0.0] * len(_FINGER_JOINTS)
        fingers.effort = [0.0] * len(_FINGER_JOINTS)
        cls.finger_pub.publish(fingers)

        twist = TwistStamped()
        twist.header.stamp = now
        twist.header.frame_id = 'pelvis'
        cls.twist_pub.publish(twist)

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'pelvis'
        pose.pose.orientation.w = 1.0
        cls.pose_pub.publish(pose)

    @classmethod
    def _publish_mock_camera(cls):
        """Publish a black 640x480 RGB8 frame + matching CameraInfo."""
        stamp = cls.node.get_clock().now().to_msg()
        frame_id = 'realsense_d435_rgb_optical_frame'

        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = frame_id
        img.height = 480
        img.width = 640
        img.encoding = 'rgb8'
        img.is_bigendian = 0
        img.step = img.width * 3
        img.data = cls._camera_frame
        cls.camera_pub.publish(img)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.height = 480
        info.width = 640
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [600.0, 0.0, 320.0, 0.0, 0.0, 600.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        cls.camera_info_pub.publish(info)

    def _call_trigger(self, client, service_name, timeout_s=10.0):
        self.assertTrue(
            client.wait_for_service(timeout_sec=timeout_s),
            f'{service_name} service not available within {timeout_s}s',
        )
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout_s)
        resp = future.result()
        self.assertIsNotNone(resp, f'{service_name} call timed out')
        self.assertTrue(
            resp.success,
            f'{service_name} returned failure: {resp.message}',
        )
        return resp

    def _latest_recorder_state(self) -> str:
        for data in reversed(self._status_messages):
            try:
                return json.loads(data).get('state', '')
            except json.JSONDecodeError:
                continue
        return ''

    def _wait_for_recorder_state(self, target: str, timeout_s: float) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._latest_recorder_state() == target:
                return
        self.fail(
            f'Recorder did not reach state {target!r} within {timeout_s}s; '
            f'last state was {self._latest_recorder_state()!r}'
        )

    def test_records_mock_teleop_episode(self):
        """Drive the recorder through a full episode with mock teleop data."""
        wait_for_controllers(
            self, self.node, self.CONTROLLERS, self.CONTROLLER_STARTUP_WAIT_S
        )
        spin_for(
            self.node,
            self.STABILIZATION_WAIT_S,
            f'Stabilising for {self.STABILIZATION_WAIT_S}s after reset...',
        )

        start_resp = self._call_trigger(
            self.start_cli, '/g1_recorder/start_recording', timeout_s=30.0)
        episode_dir = Path(start_resp.message)

        # Encoder load takes ~2–5 s; wait for RECORDING.
        self._wait_for_recorder_state(
            'RECORDING', self.RECORDER_STATE_TIMEOUT_S)

        # Phase 1 — rest: both hands in front, open fingers.
        self.__class__._phase = _PHASE_REST
        spin_for(
            self.node,
            self.REST_PHASE_DURATION_S,
            f'Phase 1 ({self.REST_PHASE_DURATION_S}s): rest pose, '
            f'both hands open in front.',
        )

        # Phase 2 — lift right hand, close left fingers.
        self.__class__._phase = _PHASE_LIFT
        spin_for(
            self.node,
            self.LIFT_PHASE_DURATION_S,
            f'Phase 2 ({self.LIFT_PHASE_DURATION_S}s): lift right hand, '
            f'close left fingers.',
        )

        self._call_trigger(
            self.stop_cli, '/g1_recorder/stop_recording', timeout_s=10.0)
        self._wait_for_recorder_state(
            'IDLE', self.RECORDER_STATE_TIMEOUT_S)

        self.assertTrue(
            episode_dir.exists(),
            f'Episode directory missing: {episode_dir}',
        )
        task_file = episode_dir / 'task.txt'
        self.assertTrue(task_file.exists(), 'task.txt missing from episode')
        self.assertIn(
            _TASK_DESCRIPTION, task_file.read_text(),
            'task.txt does not contain the configured task description',
        )

        bag_path = episode_dir / 'bag'
        self.assertTrue(bag_path.exists(), f'Bag missing: {bag_path}')

        from rosbags.rosbag2 import Reader
        # Collect receive-time (log-time) timestamps per topic from the bag
        # so we can validate both presence and steady-state publication rate.
        topic_stamps_ns: dict[str, list[int]] = {}
        with Reader(bag_path) as reader:
            for conn, timestamp_ns, _ in reader.messages():
                topic_stamps_ns.setdefault(conn.topic, []).append(timestamp_ns)

        expected_topics = list(_EXPECTED_RATES_HZ) + list(_LATCHED_TOPICS)
        missing = [t for t in expected_topics if t not in topic_stamps_ns]
        self.assertFalse(
            missing,
            f'Topics missing from bag: {missing}. '
            f'Present: {sorted(topic_stamps_ns)}',
        )

        for topic in _LATCHED_TOPICS:
            self.assertGreaterEqual(
                len(topic_stamps_ns[topic]), 1,
                f'{topic} (latched) has no messages',
            )

        rate_report = {}
        for topic, (target_hz, min_hz) in _EXPECTED_RATES_HZ.items():
            stamps = topic_stamps_ns[topic]
            # Need at least 2 samples to measure a rate; recording ran for
            # ``RECORDING_DURATION_S`` s so every non-latched topic must
            # have produced multiple messages.
            self.assertGreaterEqual(
                len(stamps), 2,
                f'{topic}: only {len(stamps)} message(s); '
                f'expected ~{target_hz} Hz over {self.RECORDING_DURATION_S}s',
            )
            duration_s = (stamps[-1] - stamps[0]) / 1e9
            self.assertGreater(
                duration_s, 0.0,
                f'{topic}: non-monotonic or zero-duration timestamps',
            )
            actual_hz = (len(stamps) - 1) / duration_s
            rate_report[topic] = round(actual_hz, 2)
            self.assertGreaterEqual(
                actual_hz, min_hz,
                f'{topic}: {actual_hz:.1f} Hz < {min_hz} Hz minimum '
                f'(target ~{target_hz} Hz)',
            )

        latched_counts = {t: len(topic_stamps_ns[t]) for t in _LATCHED_TOPICS}
        self.node.get_logger().info(
            f'Bag rates (Hz): {rate_report}; latched counts: {latched_counts}'
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Launch-testing post-shutdown placeholder."""

    def test_exit_codes(self, proc_info):
        """Ensure the launch ran to completion."""
