#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

"""Top-level bringup for Unitree G1 humanoid cloud control.

"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    vda5050_share = Path(get_package_share_directory('isaac_ros_vda5050_client_bringup'))
    vgl_model_dir_default = (
        Path(get_package_share_directory('isaac_ros_visual_mapping')) / 'models'
    )
    gr00t_agile_share = FindPackageShare('isaac_ros_unitree_g1_gr00t')

    declared_arguments = [
        # --- Hardware ---
        DeclareLaunchArgument(
            'hardware_type',
            default_value='mujoco',
            description="Hardware type: 'mujoco' or 'isaacsim' for simulation, 'real' for "
                        'physical G1.',
            choices=['mujoco', 'real', 'isaacsim'],
        ),
        DeclareLaunchArgument(
            'enable_viewer',
            default_value='true',
            description='[MuJoCo only] Enable MuJoCo viewer GUI.',
        ),
        DeclareLaunchArgument(
            'network_interface',
            default_value='eno1',
            description='[Real hardware only] Network interface for G1 communication.',
        ),
        DeclareLaunchArgument(
            'use_foxglove',
            default_value='true',
            description='Start Foxglove Studio bridge for visualization.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock (True for MuJoCo, False for real hardware).',
        ),
        # --- Policy ---
        DeclareLaunchArgument(
            'controller_group',
            default_value='gr00t_n17_apple_to_plate',
            description='Controller group from unitree_g1_bringup/config/controller_groups.yaml.',
        ),
        # --- Robot identity ---
        DeclareLaunchArgument(
            'serial_number',
            default_value='g1_0',
            description='VDA5050 robot serial number (used to construct MQTT topics).',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='pelvis',
            description='Robot base frame for TF lookups and nav2.',
        ),
        # --- Cloud / mission client ---
        DeclareLaunchArgument(
            'enable_mission_client',
            default_value='True',
            description='Launch the VDA5050 mission client + MQTT bridge + nav2 (and the '
                        'map_server feeding it). Set False to run the robot stack standalone '
                        '(GR00T/AGILE + gate + action server) without the cloud connection.',
            choices=['True', 'False'],
        ),
        # --- Nav2 / localization ---
        DeclareLaunchArgument(
            'map',
            default_value=str(vda5050_share / 'maps' / 'carter_warehouse_navigation.yaml'),
            description='Full path to nav2 map file.',
        ),
        DeclareLaunchArgument(
            'nav_params_file',
            default_value=str(vda5050_share / 'config' / 'humanoid_navigation.yaml'),
            description='Full path to nav2 params file.',
        ),
        DeclareLaunchArgument(
            'use_static_tf',
            default_value='True',
            description='Publish static map->odom TF (use True for humanoid without wheel odom). '
                        'Forced False when enable_camera_localization is True '
                        '(VSLAM owns map->odom).',
        ),
        DeclareLaunchArgument(
            'init_pose_x',
            default_value='0.0',
            description='Initial robot X position on the map (meters).',
        ),
        DeclareLaunchArgument(
            'init_pose_y',
            default_value='0.0',
            description='Initial robot Y position on the map (meters).',
        ),
        DeclareLaunchArgument(
            'init_pose_yaw',
            default_value='0.0',
            description='Initial robot yaw on the map (radians).',
        ),
        # --- Camera + localization (real hardware only) ---
        DeclareLaunchArgument(
            'enable_camera_localization',
            default_value='false',
            description='[Real hardware only] Launch the camera (RealSense D435 by default; '
                        'OAK also supported) + VSLAM + VGL for visual localization. '
                        'Requires map_dir for VGL/VSLAM map loading.',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='realsense_d435_rgb',
            description='[Camera localization] Camera name (used to find params under '
                        'isaac_ros_unitree_g1_cloud_control_bringup/params/<camera_name>/). '
                        "Names starting with 'realsense' select the RealSense path; names "
                        "starting with 'oak' select the OAK path.",
        ),
        DeclareLaunchArgument(
            'localization_cameras',
            default_value=LaunchConfiguration('camera_name'),
            description='[Camera localization] Comma-separated camera(s) feeding VSLAM/VGL. '
                        'Defaults to camera_name; only set this explicitly for multi-camera '
                        'localization where the localizer inputs differ from camera_name.',
        ),
        DeclareLaunchArgument(
            'map_dir',
            default_value='',
            description='[Camera localization] Folder containing cuvslam_map/ and cuvgl_map/ '
                        'subdirs. Required when loading VSLAM/VGL maps.',
        ),
        DeclareLaunchArgument(
            'vgl_model_dir',
            default_value=str(vgl_model_dir_default),
            description='[Camera localization] Directory containing the VGL ONNX models and '
                        'TensorRT engines. Uses the installed default when omitted.',
        ),
        DeclareLaunchArgument(
            'perception_start_delay_s',
            default_value='10.0',
            description='[Camera localization] Seconds to wait for the camera and TF tree '
                        'before starting VGL and VSLAM.',
        ),
        DeclareLaunchArgument(
            'enable_vslam',
            default_value='true',
            description='[Camera localization] Enable VSLAM (visual odometry + optional SLAM).',
        ),
        DeclareLaunchArgument(
            'enable_vgl',
            default_value='true',
            description='[Camera localization] Enable Visual Global Localization '
                        'against map_dir/cuvgl_map.',
        ),
        DeclareLaunchArgument(
            'enable_nvblox',
            default_value='false',
            description='[Camera localization] Enable nvblox 3D reconstruction.',
        ),
    ]

    # Active iff running real hardware AND user opted in.
    cam_loc_active = PythonExpression([
        "'", LaunchConfiguration('hardware_type'), "' == 'real' and '",
        LaunchConfiguration('enable_camera_localization'), "'.lower() == 'true'"
    ])

    # The static-TF helpers are per-camera-family. OAK ships as a single
    # camera_frame with two optical children; the RealSense ROS driver
    # publishes its own internal stereo/color geometry from <cam>_link.
    oak_cam_active = PythonExpression([
        "'", LaunchConfiguration('hardware_type'), "' == 'real' and '",
        LaunchConfiguration('enable_camera_localization'), "'.lower() == 'true' and '",
        LaunchConfiguration('camera_name'), "'.startswith('oak')"
    ])
    realsense_cam_active = PythonExpression([
        "'", LaunchConfiguration('hardware_type'), "' == 'real' and '",
        LaunchConfiguration('enable_camera_localization'), "'.lower() == 'true' and '",
        LaunchConfiguration('camera_name'), "'.startswith('realsense')"
    ])

    # When camera localization is on, VSLAM owns map->odom — force off the static TF.
    use_static_tf_effective = PythonExpression([
        "'False' if ('", LaunchConfiguration('hardware_type'),
        "' == 'real' and '", LaunchConfiguration('enable_camera_localization'),
        "'.lower() == 'true') else '", LaunchConfiguration('use_static_tf'), "'"
    ])

    # When camera localization is on, point nav2 at the VSLAM odometry topic.
    # SetParameter is unconditional (one value either way) so nav2 sees a single
    # consistent odom_topic.
    odom_topic_effective = PythonExpression([
        "'/visual_slam/tracking/odometry' if ('",
        LaunchConfiguration('hardware_type'),
        "' == 'real' and '", LaunchConfiguration('enable_camera_localization'),
        "'.lower() == 'true') else '/odom'"
    ])

    # 1. GR00T + AGILE inference graph (includes initial_noise_publisher).
    #    Deploy outputs redirected to /deploy/* so the gate controls what reaches the robot.
    inference_graph = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gr00t_agile_share, '/launch/unitree_g1_gr00t_agile.launch.py'
        ]),
        launch_arguments={
            'hardware_type': LaunchConfiguration('hardware_type'),
            'enable_viewer': LaunchConfiguration('enable_viewer'),
            'network_interface': LaunchConfiguration('network_interface'),
            'use_foxglove': LaunchConfiguration('use_foxglove'),
            'initial_controller_group': LaunchConfiguration('controller_group'),
            'joint_commands_trajectory_output_topic': '/deploy/joint_commands_trajectory',
            'cmd_vel_output_topic': '/deploy/cmd_vel',
            # Enable the RealSense stereo IR streams only when localizing with the
            # realsense; the policy-only path stays color-only (gr00t_agile default).
            'realsense_enable_infra': realsense_cam_active,
        }.items(),
    )

    # 2. GR00T output gate — starts closed; opened by the action server on goal accept.
    gate_node = Node(
        package='isaac_ros_humanoid_task_server',
        executable='groot_output_gate_node',
        name='groot_output_gate_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'joint_commands_trajectory_input_topic': '/deploy/joint_commands_trajectory',
            'joint_commands_trajectory_output_topic': '/joint_commands_trajectory',
            'cmd_vel_input_topic': '/deploy/cmd_vel',
            'cmd_vel_output_topic': '/cmd_vel',
        }],
        output='screen',
    )

    # 3. Humanoid task action server — receives VDA5050 HumanoidTask goals,
    #    opens/closes the gate, and reports result back to the mission client.
    task_server = Node(
        package='isaac_ros_humanoid_task_server',
        executable='humanoid_task_action_server',
        name='humanoid_task_action_server',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'action_server_name': 'humanoid_task',
            'gate_service': 'groot_output_gate_node/set_active',
            'gate_heartbeat_topic': 'groot_output_gate_node/heartbeat',
            'feedback_rate_hz': 10.0,
        }],
        output='screen',
    )

    # 4. VDA5050 mission client + MQTT bridge + nav2.
    #    Wrapped in a GroupAction so SetParameter('odom_topic', ...) propagates to
    #    every nav2 node started by the include (controller_server, planner_server,
    #    behavior_server, smoother_server, ...). Matches the cloud_control_2 pattern.
    mission_client = GroupAction(
        [
            SetParameter('odom_topic', odom_topic_effective),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(vda5050_share / 'launch' /
                        'isaac_ros_vda5050_client_humanoid_in_sim.launch.py')
                ),
                launch_arguments={
                    'serial_number': LaunchConfiguration('serial_number'),
                    'base_frame': LaunchConfiguration('base_frame'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'use_static_tf': use_static_tf_effective,
                    'init_pose_x': LaunchConfiguration('init_pose_x'),
                    'init_pose_y': LaunchConfiguration('init_pose_y'),
                    'init_pose_yaw': LaunchConfiguration('init_pose_yaw'),
                    'map': LaunchConfiguration('map'),
                    'nav_params_file': LaunchConfiguration('nav_params_file'),
                }.items(),
            ),
        ],
        condition=IfCondition(LaunchConfiguration('enable_mission_client')),
    )

    # 5. Camera + localization (OAK + VSLAM + VGL + nvblox).
    cam_loc_share = FindPackageShare('isaac_ros_unitree_g1_cloud_control_bringup')

    # RealSense D435 publishes IR on /<cam>/infra{1,2}/image_rect_raw rather
    # than the /<cam>/{left,right}/image_raw convention VSLAM/VGL assume,
    # and realsense2_camera 4.57 ignores ComposableNode remappings on those
    # streams. Override the subscriber-side topic names instead of fighting
    # the driver. Defaults below match OAK and pass through unchanged.
    is_realsense = PythonExpression(
        ["'", LaunchConfiguration('camera_name'), "'.startswith('realsense')"])
    vslam_left_image_topic_suffix = PythonExpression([
        "'infra1/image_rect_raw' if ", is_realsense, " else 'left/image_raw'"])
    vslam_right_image_topic_suffix = PythonExpression([
        "'infra2/image_rect_raw' if ", is_realsense, " else 'right/image_raw'"])
    vslam_left_info_topic_suffix = PythonExpression([
        "'infra1/camera_info' if ", is_realsense, " else 'left/camera_info'"])
    vslam_right_info_topic_suffix = PythonExpression([
        "'infra2/camera_info' if ", is_realsense, " else 'right/camera_info'"])
    # VGL has a built-in topic_config_file override; point it at the realsense
    # yaml when the camera is realsense, leave empty otherwise.
    vgl_topic_config_file = PythonExpression([
        "'", PathJoinSubstitution(
            [cam_loc_share, 'params', 'realsense_d435_rgb', 'vgl_topics.yaml']),
        "' if ", is_realsense, " else ''"])
    realsense_optical_frames = 'camera_infra1_optical_frame,camera_infra2_optical_frame'
    camera_optical_frames_arg = PythonExpression([
        f"'{realsense_optical_frames}' if ", is_realsense, " else ''"])

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            cam_loc_share, '/launch/sensors/camera.launch.py'
        ]),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'enable_oak_driver': 'True',
            'launch_container': 'True',
            'publish_urdf': 'False',
        }.items(),
        condition=IfCondition(cam_loc_active),
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            cam_loc_share, '/launch/include/perception_include.launch.py'
        ]),
        launch_arguments={
            'enable_perception': 'True',
            'enable_image_decompression': 'False',
            'localization_cameras': LaunchConfiguration('localization_cameras'),
            'enable_vgl': LaunchConfiguration('enable_vgl'),
            'vgl_do_rectify_images': 'True',
            'vgl_topic_config_file': vgl_topic_config_file,
            'vslam_left_image_topic_suffix': vslam_left_image_topic_suffix,
            'vslam_right_image_topic_suffix': vslam_right_image_topic_suffix,
            'vslam_left_info_topic_suffix': vslam_left_info_topic_suffix,
            'vslam_right_info_topic_suffix': vslam_right_info_topic_suffix,
            'vslam_camera_optical_frames': camera_optical_frames_arg,
            'vgl_camera_optical_frames': camera_optical_frames_arg,
            'vgl_base_frame': 'pelvis',
            'vgl_frequency': '1.0',
            'vgl_map_dir': PathJoinSubstitution([LaunchConfiguration('map_dir'), 'cuvgl_map']),
            'vgl_model_dir': LaunchConfiguration('vgl_model_dir'),
            'enable_vslam': LaunchConfiguration('enable_vslam'),
            'odom_frame': 'odom',
            'vslam_image_qos': 'SENSOR_DATA',
            'is_sim': 'False',
            'vslam_debug_data_path': '',
            'vslam_enable_slam': 'True',
            'vslam_enable_ground_constraint_in_odometry': 'True',
            'vslam_enable_ground_constraint_in_slam': 'True',
            'vslam_load_map_folder_path': PathJoinSubstitution(
                [LaunchConfiguration('map_dir'), 'cuvslam_map']),
            'vslam_use_rectified_images': 'True',
            'enable_nvblox': LaunchConfiguration('enable_nvblox'),
            'nvblox_camera_name': LaunchConfiguration('camera_name'),
            'nvblox_global_frame': 'odom',
        }.items(),
        condition=IfCondition(cam_loc_active),
    )

    # Camera TFs. gr00t_agile already publishes the G1 URDF (which contains
    # pelvis/torso_link), so publishing the camera URDF would duplicate those
    # joints. Instead, attach the camera frames to the existing torso_link
    # via static TFs.

    # OAK: publish camera_frame and the two optical children explicitly.
    # Translation matches g1_oak_cam8.urdf.
    oak_static_tfs = GroupAction(
        [
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='oak_cam8_to_torso_tf',
                arguments=['0.06', '0.0', '0.40', '0', '0', '0',
                           'torso_link', 'oak_cam8_camera_frame'],
                output='screen',
            ),
            # Optical frame convention: x-right, y-down, z-forward.
            # Camera-frame convention (REP-103 body): x-forward, y-left, z-up.
            # Rotation (rpy in body → optical): -pi/2, 0, -pi/2.
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='oak_cam8_left_optical_tf',
                arguments=['0.0', '0.0375', '0.0',
                           '-1.5707963', '0', '-1.5707963',
                           'oak_cam8_camera_frame',
                           'oak_cam8_left_camera_optical_frame'],
                output='screen',
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='oak_cam8_right_optical_tf',
                arguments=['0.0', '-0.0375', '0.0',
                           '-1.5707963', '0', '-1.5707963',
                           'oak_cam8_camera_frame',
                           'oak_cam8_right_camera_optical_frame'],
                output='screen',
            ),
        ],
        condition=IfCondition(oak_cam_active),
    )

    # RealSense: attach the driver's default base (`camera_link`) to the robot torso
    realsense_static_tfs = GroupAction(
        [
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='realsense_camera_link_to_torso_tf',
                arguments=['0.0576235', '0.01753', '0.41987',
                           '0', '0.8307767239493009', '0',
                           'torso_link', 'camera_link'],
                output='screen',
            ),
        ],
        condition=IfCondition(realsense_cam_active),
    )

    # 6. map_server + dedicated lifecycle_manager.
    #    The bundled nav2 sub-launch (vda5050 navigation.launch.py) intentionally
    #    omits map_server from its lifecycle list, so we run our own here to
    #    publish /map from the `map` launch arg.
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': LaunchConfiguration('map'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_mission_client')),
    )
    map_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server'],
        }],
        condition=IfCondition(LaunchConfiguration('enable_mission_client')),
    )

    return LaunchDescription(
        declared_arguments + [
            inference_graph,
            gate_node,
            task_server,
            mission_client,
            camera_launch,
            oak_static_tfs,
            realsense_static_tfs,
            TimerAction(
                period=LaunchConfiguration('perception_start_delay_s'),
                actions=[perception_launch],
            ),
            map_server_node,
            map_lifecycle_manager_node,
        ]
    )
