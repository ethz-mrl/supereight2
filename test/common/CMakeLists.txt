# SPDX-FileCopyrightText: 2021-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10...3.31)

add_executable(rgb-unittest "rgb_unittest.cpp")
target_compile_options(rgb-unittest PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_link_libraries(rgb-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME rgb-unittest COMMAND rgb-unittest)
