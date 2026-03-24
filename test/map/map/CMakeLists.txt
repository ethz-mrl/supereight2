# SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10...3.31)

add_executable(map-aabb-unittest map_aabb_unittest.cpp)
target_compile_options(map-aabb-unittest PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_link_libraries(map-aabb-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME map-aabb-unittest COMMAND map-aabb-unittest)

add_executable(map-unittest map_unittest.cpp)
target_compile_options(map-unittest PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_link_libraries(map-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME map-unittest COMMAND map-unittest)
