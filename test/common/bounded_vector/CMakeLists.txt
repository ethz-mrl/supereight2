# SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10...3.31)

add_executable(bounded-vector-unittest "bounded_vector_unittest.cpp")
target_compile_options(bounded-vector-unittest PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_link_libraries(bounded-vector-unittest PRIVATE SRL::Supereight2 gtest_main)
add_test(NAME bounded-vector-unittest COMMAND bounded-vector-unittest)
