# SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10...3.31)

# Disable warnings for third-party code
add_compile_options(-w)

# SRL Projection
option(SRL_PROJECTION_BUILD_TESTS "Enable testing" OFF)
add_subdirectory(srl_projection)

if(SE_TEST AND NOT TARGET gtest AND NOT TARGET gtest_main)
    set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
    option(BUILD_GMOCK OFF)
    option(INSTALL_GTEST OFF)
    add_subdirectory(googletest)
endif()
