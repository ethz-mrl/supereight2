# SPDX-FileCopyrightText: 2023-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10...3.31)

add_executable(pinhole-camera-unittest "pinhole_camera_unittest.cpp")
target_compile_options(pinhole-camera-unittest PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_link_libraries(pinhole-camera-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME pinhole-camera-unittest COMMAND pinhole-camera-unittest)
