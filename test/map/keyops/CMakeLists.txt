# SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10...3.31)

add_executable(keyops-unittest keyops_unittest.cpp)
target_compile_options(keyops-unittest PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_link_libraries(keyops-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME keyops-unittest COMMAND keyops-unittest)
