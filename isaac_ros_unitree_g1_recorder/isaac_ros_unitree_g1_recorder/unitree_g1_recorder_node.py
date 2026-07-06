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
G1 recorder node with session management for bulk data collection.

Manages a state machine (IDLE/INITIALIZING/RECORDING/SAVING/CANCELING) with
ROS services for start/stop/cancel. Each episode gets its own MCAP bag
directory.

The H264 encoder is loaded/unloaded per episode via composition services
so the bag always captures SPS/PPS from encoder initialization.

Services:
  ~/start_recording  (Trigger)  — IDLE → INITIALIZING → RECORDING
  ~/stop_recording   (Trigger)  — RECORDING → SAVING → IDLE
  ~/cancel_recording (Trigger)  — RECORDING/INITIALIZING → CANCELING → IDLE

Status published on ~/status at 2 Hz as JSON string.
"""

from datetime import datetime
import json
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time

from composition_interfaces.srv import LoadNode, UnloadNode
from geometry_msgs.msg import PoseStamped, TwistStamped
from isaac_ros_data_flywheel.msg import RecordData, TopicStamp
from isaac_ros_deploy_interfaces.msg import JointCommand
from rcl_interfaces.msg import (
    Parameter as ParameterMsg,
    ParameterType,
    ParameterValue,
    SetParametersResult,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Imu, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


_TOPIC_RECORD_DATA = '/record_data'
_TOPIC_ROOT_TWIST = '/xr_teleop/root_twist'
_TOPIC_ROOT_POSE = '/xr_teleop/root_pose'
_TOPIC_EE_POSES = '/xr_teleop/ee_poses'
_TOPIC_FINGER_JOINTS = '/xr_teleop/finger_joints'
_TOPIC_TASK_DESC = '/recording/task_description'
_TOPIC_ROBOT_DESCRIPTION = '/robot_description'
_TOPIC_TF = '/tf'
_TOPIC_TF_STATIC = '/tf_static'
_TOPIC_JOINT_COMMANDS = '/applied_joint_commands'


class UnitreeG1RecorderNode(Node):
    """Session-managed recorder for G1 teleop demonstrations."""

    def __init__(self, **kwargs):
        super().__init__('g1_recorder', **kwargs)

        # --- Parameters ---
        self.declare_parameter('task_description', '')
        self.declare_parameter('sync_rate', 30.0)
        self.declare_parameter('camera_topic', '/camera/color/image_compressed')
        self.declare_parameter('camera_raw_topic',
                               '/realsense_d435_rgb/color/image_raw')
        self.declare_parameter('camera_info_topic',
                               '/realsense_d435_rgb/color/camera_info')
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('imu_topic', '/imu_sensor_broadcaster/imu')
        self.declare_parameter('output_dir', '/workspaces/isaac_ros-dev/recordings')
        self.declare_parameter('encoder_container_name',
                               '/h264_encoder_container')

        task_description = self.get_parameter('task_description').value
        self._sync_rate = self.get_parameter('sync_rate').value
        self._camera_topic = self.get_parameter('camera_topic').value
        self._camera_raw_topic = self.get_parameter('camera_raw_topic').value
        self._camera_info_topic = self.get_parameter('camera_info_topic').value
        self._camera_width = self.get_parameter('camera_width').value
        self._camera_height = self.get_parameter('camera_height').value
        self._joint_states_topic = self.get_parameter('joint_states_topic').value
        self._imu_topic = self.get_parameter('imu_topic').value
        base_dir = Path(self.get_parameter('output_dir').value).expanduser()
        session_name = f'session_{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}'
        self._output_dir = base_dir / session_name
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._encoder_container = self.get_parameter(
            'encoder_container_name').value

        if not task_description:
            raise RuntimeError(
                "Parameter 'task_description' is required and must not be empty.\n"
                "Pass it as a launch argument: task_description:='your task here'"
            )
        if self._sync_rate <= 0:
            raise RuntimeError(
                f"Parameter 'sync_rate' must be positive, got {self._sync_rate}"
            )

        # Register parameter validation after initial declaration/validation
        # so the default empty value above does not trigger the callback.
        self.add_on_set_parameters_callback(self._validate_parameters)

        # --- State ---
        # IDLE | INITIALIZING | RECORDING | SAVING | CANCELING
        self._state = 'IDLE'
        self._saved_episode_count = 0
        self._episode_counter = 0
        self._current_episode_path = ''
        self._recording_start_time = None
        self._bag_process = None
        self._bag_log_file = None
        self._pending_unload_future = None
        self._encoder_unique_id = None
        self._cleanup_thread = None
        self._cleanup_action = None
        self._cleanup_pending_action = None

        # Topics to record in each bag.
        # /robot_description and /tf_static are latched (TRANSIENT_LOCAL) —
        # ros2 bag record adopts the publisher's durability so they are
        # captured even though they were sent before the bag started.
        self._topics_to_record = [
            self._joint_states_topic,
            _TOPIC_JOINT_COMMANDS,
            _TOPIC_RECORD_DATA,
            self._camera_topic,
            self._camera_info_topic,
            _TOPIC_ROOT_TWIST,
            _TOPIC_ROOT_POSE,
            _TOPIC_EE_POSES,
            _TOPIC_FINGER_JOINTS,
            self._imu_topic,
            _TOPIC_TASK_DESC,
            _TOPIC_ROBOT_DESCRIPTION,
            _TOPIC_TF,
            _TOPIC_TF_STATIC,
        ]

        # --- Latest-message stamps per topic ---
        self._joint_states_stamp = None
        self._imu_stamp = None
        self._camera_stamp = None
        self._joint_commands_stamp = None
        self._root_twist_stamp = None
        self._root_pose_stamp = None

        self._warn_tick = 0
        self._warn_interval = max(1, int(self._sync_rate * 5))

        # --- Subscriptions ---
        self.create_subscription(
            JointState, self._joint_states_topic,
            self._on_joint_states, qos_profile_sensor_data)
        self.create_subscription(
            Imu, self._imu_topic,
            self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            CompressedImage, self._camera_topic,
            self._on_camera, qos_profile_sensor_data)
        self.create_subscription(
            JointCommand, _TOPIC_JOINT_COMMANDS,
            self._on_joint_commands, qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, _TOPIC_ROOT_TWIST,
            self._on_root_twist, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, _TOPIC_ROOT_POSE,
            self._on_root_pose, qos_profile_sensor_data)

        # --- Publishers ---
        # These are created per-episode in _create_episode_publishers()
        # so DDS announces them AFTER the bag starts (otherwise the bag
        # subprocess misses the announcements and never subscribes).
        self._record_data_pub = None
        self._task_pub = None

        self._status_pub = self.create_publisher(String, '~/status', 10)

        # --- Composition service clients ---
        self._load_client = self.create_client(
            LoadNode, f'{self._encoder_container}/_container/load_node')
        self._unload_client = self.create_client(
            UnloadNode, f'{self._encoder_container}/_container/unload_node')

        # --- Services ---
        self.create_service(Trigger, '~/start_recording',
                            self._on_start_recording)
        self.create_service(Trigger, '~/stop_recording',
                            self._on_stop_recording)
        self.create_service(Trigger, '~/cancel_recording',
                            self._on_cancel_recording)

        # --- Timers ---
        self.create_timer(1.0 / self._sync_rate, self._on_sync_timer)
        self.create_timer(0.5, self._on_status_timer)
        # Poll timer for INITIALIZING → RECORDING transition.
        self._init_timer = self.create_timer(0.1, self._on_init_poll)
        self._init_timer.cancel()
        self._encoder_load_future = None
        self._bag_ready_time = 0.0

        # Cleanup timer for async stop/cancel (avoids blocking service callbacks).
        self._cleanup_timer = self.create_timer(0.1, self._on_cleanup)
        self._cleanup_timer.cancel()
        # Poll timer that finalizes cleanup once the bag-stop worker exits.
        self._cleanup_poll_timer = self.create_timer(
            0.1, self._on_cleanup_poll)
        self._cleanup_poll_timer.cancel()

        self.get_logger().info(
            f'G1 recorder started in IDLE state.\n'
            f'  task        : {task_description!r}\n'
            f'  sync_rate   : {self._sync_rate} Hz\n'
            f'  camera_topic: {self._camera_topic}\n'
            f'  output_dir  : {self._output_dir}\n'
            f'Waiting for ~/start_recording service call...'
        )

    # --- Properties ---

    @property
    def state(self) -> str:
        return self._state

    @property
    def saved_episode_count(self) -> int:
        return self._saved_episode_count

    # --- Parameter validation ---

    def _validate_parameters(self, params):
        """Reject parameter changes that would leave the node in a bad state."""
        for param in params:
            if param.name == 'task_description':
                if not isinstance(param.value, str) or not param.value.strip():
                    return SetParametersResult(
                        successful=False,
                        reason="'task_description' must be a non-empty string",
                    )
        return SetParametersResult(successful=True)

    # --- Service callbacks ---

    def _on_start_recording(self, request, response):
        if self._state != 'IDLE':
            response.success = False
            response.message = f'Cannot start: currently in {self._state} state'
            return response

        self._episode_counter += 1
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        episode_name = f'episode_{self._episode_counter:03d}_{timestamp}'
        episode_path = self._output_dir / episode_name
        episode_path.mkdir(parents=True, exist_ok=True)
        self._current_episode_path = str(episode_path)

        # Write task file.
        task_description = self.get_parameter('task_description').value
        task_file = episode_path / 'task.txt'
        task_file.write_text(
            f'task: {task_description}\n'
            f'recorded_at: {datetime.now().isoformat()}\n'
        )

        # Start bag recording first.  The encoder is loaded AFTER the bag
        # has subscribed to topics, so the bag captures SPS/PPS from the
        # encoder's very first output.  The _on_init_poll timer handles
        # the two-phase startup: wait for bag → load encoder → RECORDING.
        bag_dir = episode_path / 'bag'
        bag_log = episode_path / 'bag_record.log'
        self._bag_log_file = bag_log.open('w')
        # Propagate the node's sim-time setting to the bag subprocess so
        # bag log_time stamps stay aligned with message header stamps when
        # recording from a simulator that publishes /clock.
        use_sim_time = bool(self.get_parameter('use_sim_time').value)
        bag_cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag_dir)]
        if use_sim_time:
            bag_cmd.append('--use-sim-time')
        bag_cmd += ['--topics', *self._topics_to_record]
        try:
            self._bag_process = subprocess.Popen(
                bag_cmd,
                stdout=subprocess.DEVNULL,
                stderr=self._bag_log_file,
            )
        except OSError as e:
            self._bag_log_file.close()
            self._bag_log_file = None
            shutil.rmtree(episode_path, ignore_errors=True)
            self._current_episode_path = ''
            response.success = False
            response.message = f'Failed to launch ros2 bag: {e}'
            self.get_logger().error(response.message)
            return response

        # Reset all stamps so sync timer waits for fresh data.
        self._joint_states_stamp = None
        self._imu_stamp = None
        self._camera_stamp = None
        self._joint_commands_stamp = None
        self._root_twist_stamp = None
        self._root_pose_stamp = None
        self._warn_tick = 0

        self._state = 'INITIALIZING'
        self._bag_ready_time = time.monotonic() + 2.0  # wait for bag DDS
        self._encoder_load_future = None  # phase 1: wait, phase 2: encoder
        self._init_timer.reset()

        self.get_logger().info(
            f'Initializing episode {self._episode_counter}: {episode_path}'
        )
        response.success = True
        response.message = str(episode_path)
        return response

    def _on_stop_recording(self, request, response):
        if self._state != 'RECORDING':
            response.success = False
            response.message = f'Cannot stop: currently in {self._state} state'
            return response

        # Transition to SAVING and return immediately. Cleanup happens
        # asynchronously in _on_cleanup so we don't block the executor.
        self._state = 'SAVING'
        self._cleanup_action = 'save'
        self._cleanup_timer.reset()
        response.success = True
        response.message = self._current_episode_path
        return response

    def _on_cancel_recording(self, request, response):
        if self._state not in ('RECORDING', 'INITIALIZING'):
            response.success = False
            response.message = f'Cannot cancel: currently in {self._state} state'
            return response

        self._state = 'CANCELING'
        self._cleanup_action = 'cancel'
        self._cleanup_timer.reset()
        response.success = True
        response.message = 'Recording canceled'
        return response

    def _on_cleanup(self):
        """
        Begin async cleanup: SIGINT bag, wait on a worker, finalize via poll.

        Dispatches blocking ``proc.wait`` to a worker thread so the executor
        stays responsive (a slow ``ros2 bag record`` shutdown would otherwise
        freeze service callbacks, timers, and subscriptions for up to 5 s).
        """
        self._cleanup_timer.cancel()
        action = self._cleanup_action
        self._cleanup_action = None
        if action is None:
            return

        self._init_timer.cancel()
        self._resolve_inflight_encoder_load()
        self._unload_encoder()
        self._signal_bag_process()

        self._cleanup_pending_action = action
        self._cleanup_thread = threading.Thread(
            target=self._wait_bag_process, daemon=True)
        self._cleanup_thread.start()
        self._cleanup_poll_timer.reset()

    def _on_cleanup_poll(self):
        """Wait for the cleanup worker thread, then finalize on executor."""
        thread = self._cleanup_thread
        if thread is not None and thread.is_alive():
            return
        self._cleanup_poll_timer.cancel()
        self._cleanup_thread = None

        self._finalize_bag_process_state()
        action = self._cleanup_pending_action
        self._cleanup_pending_action = None
        self._finalize_cleanup(action)

    def _finalize_cleanup(self, action):
        """Transition to IDLE once the bag process has exited."""
        if action == 'save':
            self._saved_episode_count += 1
            if self._recording_start_time is not None:
                duration = time.monotonic() - self._recording_start_time
                duration_str = f' ({duration:.1f}s)'
            else:
                duration_str = ''
            self.get_logger().info(
                f'Saved episode {self._episode_counter}'
                f'{duration_str}: {self._current_episode_path}'
            )
            self._current_episode_path = ''
        elif action == 'cancel':
            episode_path = Path(self._current_episode_path)
            if episode_path.exists():
                shutil.rmtree(episode_path)
                self.get_logger().info(f'Canceled and deleted: {episode_path}')
            self._current_episode_path = ''

        self._destroy_episode_publishers()
        self._recording_start_time = None
        self._state = 'IDLE'

    def _resolve_inflight_encoder_load(self):
        """Handle in-flight encoder load future during cleanup."""
        if self._encoder_load_future is None:
            return
        if self._encoder_load_future.done():
            try:
                result = self._encoder_load_future.result()
            except Exception:
                result = None
            if result and result.success:
                self._encoder_unique_id = result.unique_id
        else:
            self._encoder_load_future.add_done_callback(
                self._on_late_encoder_load)
        self._encoder_load_future = None

    # --- Per-episode publishers ---

    def _create_episode_publishers(self):
        """Create publishers that the bag needs to record."""
        self._record_data_pub = self.create_publisher(
            RecordData, _TOPIC_RECORD_DATA,
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=1))
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        self._task_pub = self.create_publisher(
            String, _TOPIC_TASK_DESC, latched_qos)
        # Publish task description now (latched for the bag).
        task_msg = String()
        task_msg.data = self.get_parameter('task_description').value
        self._task_pub.publish(task_msg)

    def _destroy_episode_publishers(self):
        """Destroy per-episode publishers."""
        for pub in (self._record_data_pub, self._task_pub):
            if pub:
                self.destroy_publisher(pub)
        self._record_data_pub = None
        self._task_pub = None

    # --- Encoder lifecycle ---

    def _load_encoder(self):
        """Asynchronously load the H264 encoder into the composable container."""
        req = LoadNode.Request()
        req.package_name = 'isaac_ros_h264_encoder'
        req.plugin_name = (
            'nvidia::isaac_ros::h264_encoder::EncoderNode')
        req.node_name = 'h264_encoder'
        req.parameters = [
            ParameterMsg(
                name='input_width',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER,
                    integer_value=self._camera_width)),
            ParameterMsg(
                name='input_height',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER,
                    integer_value=self._camera_height)),
        ]
        req.remap_rules = [
            f'image_raw:={self._camera_raw_topic}',
            f'image_compressed:={self._camera_topic}',
        ]
        self._encoder_load_future = self._load_client.call_async(req)
        self._init_timer.reset()

    def _on_init_poll(self):
        """
        Two-phase init: wait for bag DDS, then load encoder.

        Phase 1: _encoder_load_future is None — waiting for bag to subscribe.
        Phase 2: _encoder_load_future is set — waiting for encoder to load.
        """
        if self._state != 'INITIALIZING':
            self._init_timer.cancel()
            return

        # Phase 1: wait for bag DDS + previous encoder unload.
        if self._encoder_load_future is None:
            # Wait for any pending unload from the previous episode.
            if self._pending_unload_future is not None:
                if not self._pending_unload_future.done():
                    return
                self._pending_unload_future = None
            if self._bag_process and self._bag_process.poll() is not None:
                self.get_logger().error('Bag process died during init.')
                self._stop_bag_process()
                ep = Path(self._current_episode_path)
                if ep.exists():
                    shutil.rmtree(ep)
                self._current_episode_path = ''
                self._state = 'IDLE'
                self._init_timer.cancel()
                return
            if time.monotonic() < self._bag_ready_time:
                return
            # Create episode publishers now so the bag discovers them
            # via fresh DDS announcements (publishers created in __init__
            # are announced before the bag starts and get missed).
            self._create_episode_publishers()
            self.get_logger().info('Bag ready, loading encoder...')
            self._load_encoder()
            return

        # Phase 2: wait for encoder load to complete.
        # Check bag health first — without this we could happily activate
        # the encoder into a dead bag and transition to RECORDING silently.
        if self._bag_process and self._bag_process.poll() is not None:
            self.get_logger().error('Bag process died while loading encoder.')
            self._init_timer.cancel()
            self._state = 'CANCELING'
            self._cleanup_action = 'cancel'
            self._cleanup_timer.reset()
            return
        if not self._encoder_load_future.done():
            return

        self._init_timer.cancel()
        try:
            result = self._encoder_load_future.result()
        except Exception as e:
            self.get_logger().error(f'Encoder load future failed: {e}')
            result = None
        self._encoder_load_future = None

        if result is not None and result.success:
            self._encoder_unique_id = result.unique_id
            self.get_logger().info(
                f'Encoder loaded (id={result.unique_id}). Recording.'
            )
            self._recording_start_time = time.monotonic()
            self._state = 'RECORDING'
        else:
            err = result.error_message if result else 'service call failed'
            self.get_logger().error(
                f'Failed to load encoder: {err}. Aborting episode.')
            # Route through the async cleanup pipeline so the blocking
            # ``proc.wait`` on the bag subprocess runs off the executor
            # thread and the per-episode publishers created in Phase 1
            # get destroyed via ``_finalize_cleanup``.
            self._state = 'CANCELING'
            self._cleanup_action = 'cancel'
            self._cleanup_timer.reset()

    def _on_late_encoder_load(self, future):
        """
        Unload an encoder whose load completed after cleanup discarded it.

        Unloads directly by unique_id without touching self._encoder_unique_id,
        which may already belong to a new episode's encoder.
        """
        try:
            result = future.result()
            if result and result.success:
                self.get_logger().info(
                    f'Late encoder load completed (id={result.unique_id}), '
                    'unloading.')
                req = UnloadNode.Request()
                req.unique_id = result.unique_id
                self._unload_client.call_async(req)
        except Exception as e:
            self.get_logger().warn(f'Late encoder load error: {e}')

    def _unload_encoder(self):
        """
        Unload the encoder from the container (fire-and-forget).

        Called from timer callbacks on the single-threaded executor, so we
        can't block waiting for the response. The unload is best-effort.
        """
        if self._encoder_unique_id is None:
            return
        req = UnloadNode.Request()
        req.unique_id = self._encoder_unique_id
        uid = self._encoder_unique_id
        self._encoder_unique_id = None

        def _on_unload_done(future):
            try:
                result = future.result()
                if result and not result.success:
                    self.get_logger().warn(
                        f'Encoder unload failed: {result.error_message}')
                else:
                    self.get_logger().info(f'Encoder unloaded (id={uid}).')
            except Exception as e:
                self.get_logger().warn(f'Encoder unload error: {e}')

        future = self._unload_client.call_async(req)
        future.add_done_callback(_on_unload_done)
        self._pending_unload_future = future

    # --- Recording resource management ---

    def _signal_bag_process(self):
        """Send SIGINT to the bag process (non-blocking). No-op if absent."""
        proc = self._bag_process
        if proc is None or proc.poll() is not None:
            return
        proc.send_signal(signal.SIGINT)

    def _wait_bag_process(self):
        """
        Block until the bag process exits. Escalates to SIGKILL on timeout.

        Runs on a worker thread during async cleanup; also used directly from
        synchronous shutdown paths (main finally, init-failure abort).
        """
        proc = self._bag_process
        if proc is None:
            return
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

    def _finalize_bag_process_state(self):
        """Close the bag log file and drop the process reference (executor thread)."""
        proc = self._bag_process
        self._bag_process = None
        if self._bag_log_file:
            self._bag_log_file.close()
            self._bag_log_file = None
        if proc is not None and proc.returncode not in (0, -signal.SIGINT):
            self.get_logger().warn(
                f'ros2 bag record exited with code {proc.returncode}')

    def _stop_bag_process(self):
        """Blocking stop: SIGINT, wait, then finalize state. Up to ~5 s."""
        self._signal_bag_process()
        self._wait_bag_process()
        self._finalize_bag_process_state()

    # --- Subscription callbacks ---

    def _on_joint_states(self, msg: JointState):
        self._joint_states_stamp = (msg.header.stamp, self.get_clock().now())

    def _on_imu(self, msg: Imu):
        self._imu_stamp = (msg.header.stamp, self.get_clock().now())

    def _on_camera(self, msg: CompressedImage):
        self._camera_stamp = (msg.header.stamp, self.get_clock().now())

    def _on_joint_commands(self, msg: JointCommand):
        self._joint_commands_stamp = (
            msg.header.stamp, self.get_clock().now())

    def _on_root_twist(self, msg: TwistStamped):
        self._root_twist_stamp = (msg.header.stamp, self.get_clock().now())

    def _on_root_pose(self, msg: PoseStamped):
        self._root_pose_stamp = (msg.header.stamp, self.get_clock().now())

    # --- Sync timer ---

    def _on_sync_timer(self):
        if self._state != 'RECORDING' or self._record_data_pub is None:
            return

        # Bail out if the bag subprocess died under us — otherwise we keep
        # accumulating sync messages into a dead sink until the operator
        # notices and calls stop.
        if self._bag_process and self._bag_process.poll() is not None:
            self.get_logger().error(
                'Bag process died mid-recording; auto-canceling episode.')
            self._state = 'CANCELING'
            self._cleanup_action = 'cancel'
            self._cleanup_timer.reset()
            return

        latest = {
            self._joint_states_topic: self._joint_states_stamp,
            self._imu_topic: self._imu_stamp,
            self._camera_topic: self._camera_stamp,
            _TOPIC_JOINT_COMMANDS: self._joint_commands_stamp,
            _TOPIC_ROOT_TWIST: self._root_twist_stamp,
            _TOPIC_ROOT_POSE: self._root_pose_stamp,
        }

        missing = [t for t, v in latest.items() if v is None]
        if missing:
            self._warn_tick += 1
            if self._warn_tick % self._warn_interval == 0:
                self.get_logger().warn(
                    f'Waiting for first message on: {missing}')
            return

        now = self.get_clock().now()

        topic_stamps = []
        for topic_name, (header_stamp, receive_time) in latest.items():
            delay_ms = (now - receive_time).nanoseconds / 1e6
            ts = TopicStamp()
            ts.topic_name = topic_name
            ts.stamp = header_stamp
            ts.delay_ms = delay_ms
            topic_stamps.append(ts)

        msg = RecordData()
        msg.header.stamp = now.to_msg()
        msg.topic_stamps = topic_stamps
        self._record_data_pub.publish(msg)

    # --- Status timer ---

    def _on_status_timer(self):
        status_msg = String()
        status_msg.data = self._build_status_json()
        self._status_pub.publish(status_msg)

    def _build_status_json(self) -> str:
        duration = 0.0
        if self._recording_start_time is not None:
            duration = time.monotonic() - self._recording_start_time
        return json.dumps({
            'state': self._state,
            'task': self.get_parameter('task_description').value,
            'saved_episodes': self._saved_episode_count,
            'current_episode_duration_s': round(duration, 1),
            'current_episode_path': self._current_episode_path,
            'output_dir': str(self._output_dir),
        })


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UnitreeG1RecorderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # If an async cleanup is in flight (SAVING / CANCELING), let
            # its worker finish before we duplicate the proc.wait here.
            # Otherwise both threads race on _bag_process and the poll
            # timer may fire callbacks on a half-destroyed node.
            node._cleanup_poll_timer.cancel()
            if node._cleanup_thread is not None:
                node._cleanup_thread.join(timeout=5)
                node._cleanup_thread = None
            if node._bag_process is not None:
                node._stop_bag_process()
            node._unload_encoder()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
