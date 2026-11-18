# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
# flake8: noqa: F403,F405
import isaac_ros_launch_utils as lu
from isaac_ros_launch_utils.all_types import *
from launch.conditions import IfCondition


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('enable_perception')
    args.add_arg('enable_image_decompression')
    args.add_arg('localization_cameras')
    args.add_arg('enable_vgl')
    args.add_arg('vgl_do_rectify_images')
    args.add_arg('vgl_frequency')
    args.add_arg('vgl_map_dir')
    args.add_arg('vgl_model_dir', '')
    args.add_arg('vgl_base_frame', 'base_link')
    args.add_arg('enable_vslam')
    args.add_arg('odom_frame')
    args.add_arg('vslam_image_qos')
    args.add_arg('is_sim')
    args.add_arg('vslam_debug_data_path')
    args.add_arg('vslam_enable_slam')
    args.add_arg('vslam_enable_ground_constraint_in_odometry')
    args.add_arg('vslam_enable_ground_constraint_in_slam')
    args.add_arg('vslam_load_map_folder_path', '')
    args.add_arg('vslam_use_rectified_images', False)
    # Per-camera VSLAM topic suffixes; defaults match OAK left|right/image_raw.
    # Overridden for RealSense (infra1|infra2/image_rect_raw) from the parent
    # cloud_control launch.
    args.add_arg('vslam_left_image_topic_suffix', 'left/image_raw')
    args.add_arg('vslam_right_image_topic_suffix', 'right/image_raw')
    args.add_arg('vslam_left_info_topic_suffix', 'left/camera_info')
    args.add_arg('vslam_right_info_topic_suffix', 'right/camera_info')
    # Optional VGL topic-name override yaml. When set, replaces VGL's
    # auto-built /<cam>/{left,right}/image_(raw|rect) remappings with
    # explicit topic names from the yaml — used for RealSense IR streams
    # whose native topics don't match VGL's default convention.
    args.add_arg('vgl_topic_config_file', '')
    # Comma-separated list of optical frame names (one per camera, left/right
    # interleaved). Needed when the camera_info messages follow the stereo
    # convention of pointing at the left frame for both cameras — otherwise
    # VSLAM and VGL collapse the stereo pair into a single TF.
    args.add_arg('vslam_camera_optical_frames', '')
    args.add_arg('vgl_camera_optical_frames', '')
    # Nvblox parameters
    args.add_arg('enable_nvblox', False)
    args.add_arg('nvblox_camera_name', 'oak_cam8')
    args.add_arg('nvblox_global_frame', 'horizontal_frame')

    # Combine conditions for perception + vgl/vslam
    perception_and_vgl = lu.AndSubstitution(
        lu.is_true(args.enable_perception),
        lu.is_true(args.enable_vgl)
    )
    perception_and_vslam = lu.AndSubstitution(
        lu.is_true(args.enable_perception),
        lu.is_true(args.enable_vslam)
    )
    perception_and_no_vslam = lu.AndSubstitution(
        lu.is_true(args.enable_perception),
        lu.is_false(args.enable_vslam)
    )
    perception_and_nvblox = lu.AndSubstitution(
        lu.is_true(args.enable_perception),
        lu.is_true(args.enable_nvblox)
    )

    actions = args.get_launch_actions()

    # VGL - only when perception AND vgl are enabled
    actions.append(
        lu.include(
            'isaac_ros_visual_global_localization',
            'launch/include/visual_global_localization.launch.py',
            launch_arguments={
                'container_name': 'nova_container',
                'vgl_enabled_stereo_cameras': args.localization_cameras,
                'vgl_do_rectify_images': args.vgl_do_rectify_images,
                'vgl_map_frame': 'map',
                'vgl_base_frame': args.vgl_base_frame,
                'vgl_frequency': args.vgl_frequency,
                'vgl_map_dir': args.vgl_map_dir,
                'vgl_model_dir': args.vgl_model_dir,
                'vgl_config_dir': lu.get_path(
                    'isaac_ros_visual_mapping',
                    'configs/single_stereo_localizer'),
                'vgl_enable_debug': False,
                'publish_rectified_images': False,
                'topic_config_file': args.vgl_topic_config_file,
                'vgl_camera_optical_frames': args.vgl_camera_optical_frames,
            },
            condition=IfCondition(perception_and_vgl),
        ))

    # VSLAM - only when perception AND vslam are enabled
    actions.append(
        lu.include(
            'isaac_ros_unitree_g1_cloud_control_bringup',
            'launch/algorithms/vslam.launch.py',
            launch_arguments={
                'container_name': 'nova_container',
                'vslam_enabled_stereo_cameras': args.localization_cameras,
                'vslam_map_frame': 'map',
                'vslam_odom_frame': args.odom_frame,
                'vslam_image_qos': args.vslam_image_qos,
                'is_sim': args.is_sim,
                'vslam_enable_slam': args.vslam_enable_slam,
                'vslam_publish_map_to_odom_tf': args.vslam_enable_slam,  # Always publish for nvblox
                'vslam_enable_ground_constraint_in_odometry':
                    args.vslam_enable_ground_constraint_in_odometry,
                'vslam_enable_ground_constraint_in_slam':
                    args.vslam_enable_ground_constraint_in_slam,
                'vslam_load_map_folder_path': args.vslam_load_map_folder_path,
                'vslam_use_rectified_images': args.vslam_use_rectified_images,
                'vslam_left_image_topic_suffix': args.vslam_left_image_topic_suffix,
                'vslam_right_image_topic_suffix': args.vslam_right_image_topic_suffix,
                'vslam_left_info_topic_suffix': args.vslam_left_info_topic_suffix,
                'vslam_right_info_topic_suffix': args.vslam_right_info_topic_suffix,
                'vslam_camera_optical_frames': args.vslam_camera_optical_frames,
                'output_tum_path': args.vslam_debug_data_path,
            },
            condition=IfCondition(perception_and_vslam),
        ))

    # Static transform - only when perception enabled but vslam disabled
    actions.append(
        lu.static_transform(
            'map',
            args.odom_frame,
            condition=IfCondition(perception_and_no_vslam),
        ))

    # Nvblox - only when perception AND nvblox are enabled
    actions.append(
        lu.include(
            'isaac_ros_unitree_g1_cloud_control_bringup',
            'launch/algorithms/nvblox.launch.py',
            launch_arguments={
                'container_name': 'nova_container',
                'camera_name': args.nvblox_camera_name,
                'nvblox_global_frame': args.nvblox_global_frame,
            },
            condition=IfCondition(perception_and_nvblox),
        ))

    return LaunchDescription(actions)

