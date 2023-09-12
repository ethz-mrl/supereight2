# SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10...3.16)

add_subdirectory(common)
add_subdirectory(common/bounded_vector)
add_subdirectory(common/image_utils)
add_subdirectory(common/str_utils)
add_subdirectory(integrator)
add_subdirectory(map/keyops)
add_subdirectory(map/map)
add_subdirectory(map/mesh_io)
add_subdirectory(map/meshing)
add_subdirectory(map/octree_allocator)
add_subdirectory(map/octree_visitor)
add_subdirectory(sensor)
add_subdirectory(system/multires_ofusion)
# Don't add installcheck to the subdirectories, it should be built independently of the library.
