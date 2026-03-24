# SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10...3.31)

add_executable(depth-integrator-unittest depth_integrator_unittest.cpp)
target_compile_options(depth-integrator-unittest PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_link_libraries(depth-integrator-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME depth-integrator-unittest COMMAND depth-integrator-unittest)

add_executable(ray-integrator-unittest ray_integrator_unittest.cpp)
target_compile_options(ray-integrator-unittest PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_link_libraries(ray-integrator-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME ray-integrator-unittest COMMAND ray-integrator-unittest)
