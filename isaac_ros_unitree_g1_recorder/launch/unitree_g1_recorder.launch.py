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

r"""
Launch file for the Unitree G1 teleop data recorder with session management.

This launch file is intended to be run in a second terminal while the
teleop bringup is already running:

  Terminal 1:
    ros2 launch isaac_ros_unitree_g1_teleop_bringup unitree_g1_teleop.launch.py \
        hardware_type:=real

  Terminal 2:
    ros2 run isaac_ros_unitree_g1_recorder record -- \
        task_description:='pick up the red cup and place it in the box'

The recorder starts in IDLE state. Use the keyboard controller to
start/stop/cancel recordings:
  [Space] Start / Stop & Save
  [c]     Cancel & discard
  [t]     Change task description
  [q]     Quit

What gets launched:
  1. unitree_g1_recorder_node — session manager, sync publishing, joint relay
  2. h264_encoder_container   — empty composable container; encoder loaded per-episode

The H264 encoder is loaded/unloaded per episode by the recorder node
via composition services so each bag captures SPS/PPS from encoder init.

Camera:
  Both MuJoCo and real hardware publish on /realsense_d435_rgb/color/image_raw.
  MuJoCo: configured via URDF sensor block in g1_ros2_control.urdf.xacro.
  Real hardware: RealSense driver launched by the teleop bringup.

NOTE (real hardware): set blend_ratio=1.0 before recording valid demonstrations:
    ros2 param set /safety_controller blend_ratio 1.0
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'task_description',
            description=(
                'Human-readable description of the task being demonstrated. '
                'Embedded in the rosbag and written to task.txt. Required.'
            ),
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/workspaces/isaac_ros-dev/recordings',
            description='Session base directory. Episodes saved as subdirectories. '
                        'Default is inside the workspace mount for host persistence.',
        ),
        DeclareLaunchArgument(
            'sync_rate',
            default_value='30.0',
            description='Rate (Hz) at which RecordData sync messages are published. '
                        'Defaults to 30 Hz to match the RealSense D435 RGB stream.',
        ),
        DeclareLaunchArgument(
            'camera_raw_topic',
            default_value='/realsense_d435_rgb/color/image_raw',
            description='Raw sensor_msgs/Image topic for the ego-view camera.',
        ),
        DeclareLaunchArgument(
            'camera_compressed_topic',
            default_value='/camera/color/image_compressed',
            description='H264 compressed output topic (encoder output, recorded in bag).',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/realsense_d435_rgb/color/camera_info',
            description='CameraInfo topic (intrinsics / distortion), recorded in bag.',
        ),
        DeclareLaunchArgument(
            'camera_width',
            default_value='640',
            description='Camera image width in pixels.',
        ),
        DeclareLaunchArgument(
            'camera_height',
            default_value='480',
            description='Camera image height in pixels.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu_sensor_broadcaster/imu',
            description='IMU topic providing body orientation and angular velocity.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for the recorder and its ros2 bag subprocess. '
                        'Set to "true" when recording from MuJoCo so bag log '
                        'timestamps match message header stamps.',
            choices=['true', 'false'],
        ),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context, *args, **kwargs):
    task_description = LaunchConfiguration('task_description').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    sync_rate = LaunchConfiguration('sync_rate').perform(context)
    camera_raw_topic = LaunchConfiguration('camera_raw_topic').perform(context)
    camera_compressed_topic = LaunchConfiguration('camera_compressed_topic').perform(context)
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    camera_width = LaunchConfiguration('camera_width').perform(context)
    camera_height = LaunchConfiguration('camera_height').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    )

    if not task_description:
        raise RuntimeError(
            "Launch argument 'task_description' is required.\n"
            "Example: task_description:='pick up the red cup'"
        )

    # 1. Session-managed recorder node (starts in IDLE).
    #    Encoder config is passed as params so the node can load/unload
    #    the encoder composable node per episode.
    recorder_node = Node(
        package='isaac_ros_unitree_g1_recorder',
        executable='unitree_g1_recorder_node',
        name='g1_recorder',
        parameters=[{
            'task_description': task_description,
            'sync_rate': float(sync_rate),
            'camera_topic': camera_compressed_topic,
            'camera_raw_topic': camera_raw_topic,
            'camera_info_topic': camera_info_topic,
            'camera_width': int(camera_width),
            'camera_height': int(camera_height),
            'imu_topic': imu_topic,
            'output_dir': output_dir,
            'encoder_container_name': '/h264_encoder_container',
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # 2. Empty composable container for H264 encoder.
    #    The recorder node loads/unloads the encoder per episode via
    #    composition services, so each bag captures SPS/PPS from init.
    encoder_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_mt',
        name='h264_encoder_container',
        namespace='',
        composable_node_descriptions=[],
        output='screen',
    )

    # Keyboard controller is run separately (needs its own TTY for curses):
    #   ros2 run isaac_ros_unitree_g1_recorder unitree_g1_keyboard_controller

    return [
        recorder_node,
        encoder_container,
    ]
