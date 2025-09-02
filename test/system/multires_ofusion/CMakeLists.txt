# SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10...3.16)

set(READER_SRC
  ../../../app/src/config.cpp
  ../../../app/src/reader.cpp
  ../../../app/src/reader_base.cpp
  ../../../app/src/reader_interiornet.cpp
  ../../../app/src/reader_kitti.cpp
  ../../../app/src/reader_leica.cpp
  ../../../app/src/reader_newercollege.cpp
  ../../../app/src/reader_openni.cpp
  ../../../app/src/reader_raw.cpp
  ../../../app/src/reader_tum.cpp
)

get_filename_component(CONFIG_FILE "config.yaml" ABSOLUTE)

add_executable(multires-ofusion-systemtest multires_ofusion_systemtest.cpp ${READER_SRC})
target_compile_options(multires-ofusion-systemtest PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_include_directories(multires-ofusion-systemtest PRIVATE ../../../app/include)
target_link_libraries(multires-ofusion-systemtest PRIVATE SRL::Supereight2 gtest)
if(OPENMP_FOUND)
    target_link_libraries(multires-ofusion-systemtest PUBLIC OpenMP::OpenMP_CXX)
endif()
add_test(NAME multires-ofusion-systemtest COMMAND multires-ofusion-systemtest ${CONFIG_FILE})
