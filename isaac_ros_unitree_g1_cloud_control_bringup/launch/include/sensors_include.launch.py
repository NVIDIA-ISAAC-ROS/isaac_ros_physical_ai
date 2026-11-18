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
import isaac_ros_launch_utils.all_types as lut


def generate_launch_description() -> lut.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('camera_name', '')
    args.add_arg('enable_rectification', False)
    args.add_arg('publish_urdf', False)
    args.add_arg('enable_oak_driver', False)

    actions = args.get_launch_actions()

    actions.append(
        lu.include(
            'isaac_ros_unitree_g1_cloud_control_bringup',
            'launch/sensors/camera.launch.py',
            launch_arguments={
                'camera_name': args.camera_name,
                'enable_rectification': args.enable_rectification,
                'publish_urdf': args.publish_urdf,
                'enable_oak_driver': args.enable_oak_driver,
            }
        ))

    return lut.LaunchDescription(actions)

