# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(isaac_ros_unitree_g1_gr00t)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_unitree_g1_gr00t_agile_launch
    test/test_unitree_g1_gr00t_agile_launch.py
  )
endif()

ament_package()
