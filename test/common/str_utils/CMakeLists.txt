# SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10...3.31)

add_executable(str-utils-unittest "str_utils_unittest.cpp")
target_compile_options(str-utils-unittest PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_link_libraries(str-utils-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME str-utils-unittest COMMAND str-utils-unittest)
