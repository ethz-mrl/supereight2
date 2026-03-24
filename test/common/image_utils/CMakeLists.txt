# SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10...3.31)

add_executable(image-utils-io-unittest "image_utils_io_unittest.cpp")
target_compile_options(image-utils-io-unittest PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_link_libraries(image-utils-io-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME image-utils-io-unittest COMMAND image-utils-io-unittest)
