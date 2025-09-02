# SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10...3.16)

find_package(GLUT)
find_package(OpenGL)
find_package(PCL COMPONENTS io)
find_package(PkgConfig) # For OpenNI2
if(PKG_CONFIG_FOUND)
    pkg_check_modules(OPENNI2 libopenni2)
else()
    set(OPENNI2_FOUND OFF)
endif()

# Compile the reader as a library so that it isn't recompiled for each app.
set(LIB_NAME reader)
add_library(${LIB_NAME} STATIC
  "src/config.cpp"
  "src/reader.cpp"
  "src/reader_base.cpp"
  "src/reader_interiornet.cpp"
  "src/reader_kitti.cpp"
  "src/reader_leica.cpp"
  "src/reader_newercollege.cpp"
  "src/reader_openni.cpp"
  "src/reader_raw.cpp"
  "src/reader_tum.cpp"
)
target_compile_options(${LIB_NAME} PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_include_directories(${LIB_NAME} PUBLIC include)
target_link_libraries(${LIB_NAME} PUBLIC SRL::Supereight2)
if(OPENMP_FOUND)
    target_link_libraries(${LIB_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()
if(PCL_FOUND)
  # These have to be added globaly for compatibility with PCL 1.8.1 on Ubuntu 18.04.
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  target_link_libraries(${LIB_NAME} PUBLIC ${PCL_LIBRARIES})
  target_compile_definitions(${LIB_NAME} PUBLIC "-DSE_PCL")
endif()
if(OPENNI2_FOUND)
    target_include_directories(${LIB_NAME} PRIVATE ${OPENNI2_INCLUDE_DIRS})
    target_link_libraries(${LIB_NAME} PRIVATE ${OPENNI2_LIBRARIES})
    target_compile_definitions(${LIB_NAME} PRIVATE "-DSE_OPENNI2")
endif()



# Compile the apps for each configuration in SE_APP_CONFIG.
set(SE_APP_CONFIG
  "TSDFCol Single PinholeCamera"
  "TSDFCol Multi PinholeCamera"
  "TSDF Multi OusterLidar"
  "OccupancyCol Multi PinholeCamera"
  "Occupancy Multi OusterLidar"
)
foreach(APP_CONFIG ${SE_APP_CONFIG})
  # Replace spaces with semicolons to convert the string to a CMake list.
  string(REPLACE " " ";" APP_CONFIG ${APP_CONFIG})
  # Get the field, resolution and sensor type from the list.
  list(GET APP_CONFIG 0 FIELD)
  list(GET APP_CONFIG 1 RES)
  list(GET APP_CONFIG 2 SENSOR)
  # Convert to lower case.
  string(TOLOWER ${FIELD} FIELD_LC)
  string(TOLOWER ${RES} RES_LC)
  string(TOLOWER ${SENSOR} SENSOR_LC)

  # Generate an app main be replacing the field, resolution and sensor in main.cpp and writing the
  # results in GENERATED_MAIN. Do nothing if main.cpp hasn't changed since GENERATED_MAIN was
  # written.
  set(INPUT_MAIN "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
  set(GENERATED_MAIN "${CMAKE_CURRENT_BINARY_DIR}/main_${FIELD_LC}_${RES_LC}_${SENSOR_LC}.cpp")
  if("${INPUT_MAIN}" IS_NEWER_THAN "${GENERATED_MAIN}")
    file(READ "${INPUT_MAIN}" FILE_CONTENT)
    string(REPLACE "se::TSDF" "se::${FIELD}" FILE_CONTENT "${FILE_CONTENT}")
    string(REPLACE "se::Res::Single" "se::Res::${RES}" FILE_CONTENT "${FILE_CONTENT}")
    string(REPLACE "se::PinholeCamera" "se::${SENSOR}" FILE_CONTENT "${FILE_CONTENT}")
    file(WRITE "${GENERATED_MAIN}" "${FILE_CONTENT}")
  endif()

  # Compile the app using the generated file.
  set(EXE_NAME "supereight_${FIELD_LC}_${RES_LC}_${SENSOR_LC}")
  add_executable(${EXE_NAME} ${GENERATED_MAIN} "src/montage.cpp")
  target_compile_options(${EXE_NAME} PRIVATE ${SE_PRIVATE_CXX_FLAGS})
  target_include_directories(${EXE_NAME} BEFORE PRIVATE include)
  target_link_libraries(${EXE_NAME} PRIVATE SRL::Supereight2 ${LIB_NAME})
  if(OPENMP_FOUND)
    target_link_libraries(${EXE_NAME} PRIVATE OpenMP::OpenMP_CXX)
  endif()
  # Allow handling large files in 32-bit systems
  target_compile_definitions(${EXE_NAME} PRIVATE _FILE_OFFSET_BITS=64)

  # Compile with GUI support
  if(GLUT_FOUND)
    target_link_libraries(${EXE_NAME}
      PRIVATE
        ${GLUT_LIBRARIES}
        ${OPENGL_LIBRARIES}
    )
    target_compile_definitions(${EXE_NAME} PRIVATE "-DSE_GLUT")
    if(NOT APPLE)
      target_link_libraries(${EXE_NAME} PRIVATE GL)
    endif()
  endif()
endforeach()

# build ray interface
add_executable(supereight_occupancy_multi_leicalidar_ray src/main_leica_lidar.cpp)
target_compile_options(supereight_occupancy_multi_leicalidar_ray PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_include_directories(supereight_occupancy_multi_leicalidar_ray BEFORE PRIVATE include)
target_link_libraries(supereight_occupancy_multi_leicalidar_ray PRIVATE SRL::Supereight2 ${LIB_NAME})
# Allow handling large files in 32-bit systems
target_compile_definitions(supereight_occupancy_multi_leicalidar_ray PRIVATE _FILE_OFFSET_BITS=64)

add_executable(supereight2_sizes src/supereight2_sizes.cpp)
target_compile_options(supereight2_sizes PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_link_libraries(supereight2_sizes PRIVATE SRL::Supereight2 ${LIB_NAME})
