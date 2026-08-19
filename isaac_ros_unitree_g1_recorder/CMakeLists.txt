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

cmake_minimum_required(VERSION 3.16)
project(isaac_ros_unitree_g1_recorder)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/unitree_g1_recorder_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME unitree_g1_recorder_node
)

install(PROGRAMS
  scripts/unitree_g1_keyboard_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME unitree_g1_keyboard_controller
)

install(PROGRAMS
  scripts/record.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME record
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  set(_test_library_dirs
    /opt/ros/$ENV{ROS_DISTRO}/opt/sdformat_vendor/lib
  )

  # Discover and register the ament_copyright / ament_flake8 / ament_pep257
  # tests (test/test_copyright.py etc.) as well as any other lint tests
  # whose ament_* packages are declared as test_depends in package.xml.
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_recorder_state_machine
    test/test_recorder_state_machine.py
    APPEND_LIBRARY_DIRS ${_test_library_dirs}
    TIMEOUT 60
  )
  ament_add_pytest_test(test_recorder_e2e
    test/test_recorder_e2e.py
    APPEND_LIBRARY_DIRS ${_test_library_dirs}
    TIMEOUT 120
  )

  # Full-stack integration test: MuJoCo + G1 controllers + recorder.
  # Requires MuJoCo simulation and NVENC for the H264 encoder.
  find_package(mujoco_ros2_control QUIET)
  if(mujoco_ros2_control_FOUND)
    add_launch_test(test/test_recorder_with_mujoco.launch.py
      APPEND_LIBRARY_DIRS ${_test_library_dirs}
      RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
      TIMEOUT 300
    )
  endif()
endif()

ament_package()
