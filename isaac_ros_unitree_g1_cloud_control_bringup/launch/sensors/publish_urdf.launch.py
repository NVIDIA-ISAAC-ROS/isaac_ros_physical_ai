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

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut


def publish_urdf(args: lu.ArgumentContainer) -> list[lut.Node]:
    urdf_content = ''
    urdf_file = lu.get_path('isaac_ros_unitree_g1_cloud_control_bringup', f'urdf/{args.urdf_file}')

    if not urdf_file.exists():
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    try:
        with open(urdf_file, 'r') as f:
            urdf_content = f.read()
    except IOError as e:
        raise RuntimeError(f"Failed to read URDF file {urdf_file}: {e}") from e

    actions = [
        lu.log_info(['Reading URDF file from: ', str(urdf_file)]),
        lut.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}]
        )]
    return actions


def generate_launch_description() -> lu.LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('urdf_file')
    args.add_opaque_function(publish_urdf)
    actions = args.get_launch_actions()

    return lut.LaunchDescription(actions)
