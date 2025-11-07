# SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

cmake_minimum_required(VERSION 3.10...3.16)
project(Supereight2 VERSION 0.27.1 LANGUAGES CXX)

option(SE_OPENMP "Compile supereight with OpenMP" ON)
option(SE_TEST "Compile the supereight tests" ON)
option(SE_APP "Compile the supereight application" ON)
option(SE_ASSERTS "Compile supereight with runtime assertions enabled" OFF)

# Find dependencies
find_package(Eigen3 3.3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(Boost REQUIRED) # We only need Boost.Pool which is a header-only library.
find_package(TBB COMPONENTS tbb)
if(TBB_tbb_FOUND)
    set(SE_TBB 1)
endif()
find_library(MATH_LIB m)

# Compiler options
set(SUPPORTED_COMPILER "$<OR:$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>")
set(SE_PRIVATE_CXX_FLAGS "$<${SUPPORTED_COMPILER}:-Wall;-Wextra;-Wpedantic;-Wno-unknown-pragmas>")

# Compile the dependencies
add_subdirectory(third_party)

# Compile the supereight library
set(LIB_NAME "supereight2")
add_library(${LIB_NAME} STATIC
    "src/common/colour_utils.cpp"
    "src/common/system_utils.cpp"
    "src/common/id.cpp"
    "src/common/image_utils.cpp"
    "src/common/perfstats.cpp"
    "src/common/str_utils.cpp"
    "src/common/yaml.cpp"
    "src/image/image.cpp"
    "src/integrator/uncertainty.cpp"
    "src/map/data.cpp"
    "src/map/io/mesh_io.cpp"
    "src/map/preprocessor.cpp"
    "src/map/raycaster.cpp"
    "src/sensor/lidar.cpp"
    "src/sensor/range_image_lidar.cpp"
    "src/sensor/pinhole_camera.cpp"
    "src/tracker/icp.cpp"
    "src/tracker/tracker.cpp"
)
configure_file("include/se/supereight_config.hpp.in" "include/se/supereight_config.hpp" @ONLY)
# Set the C++ standard required by the library.
target_compile_features(${LIB_NAME} PUBLIC cxx_std_17)
target_compile_options(${LIB_NAME} PRIVATE ${SE_PRIVATE_CXX_FLAGS})
target_include_directories(${LIB_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${LIB_NAME}
    PUBLIC
      Eigen3::Eigen
      Boost::boost
      ${OpenCV_LIBS}
      ${TBB_IMPORTED_TARGETS}
      SRL::Projection
)
if(MATH_LIB)
    target_link_libraries(${LIB_NAME} PUBLIC ${MATH_LIB})
endif()
# Find OpenMP, warn if disabled.
if(SE_OPENMP)
    find_package(OpenMP)
    if(OPENMP_FOUND)
        target_link_libraries(${LIB_NAME} PUBLIC OpenMP::OpenMP_CXX)
        message(STATUS "Compiling with OpenMP support")
    else()
        message(WARNING "OpenMP not found. Performance may be terrible.")
    endif()
else()
    message(WARNING "Building without OpenMP. Performance may be terrible.")
endif()
if(SE_ASSERTS)
    target_compile_options(${LIB_NAME} PUBLIC -UNDEBUG)
endif()
if(("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang") AND ("${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 9.0))
    target_link_libraries(${LIB_NAME} PUBLIC stdc++fs)
elseif(("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU") AND ("${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS 9.1))
    target_link_libraries(${LIB_NAME} PUBLIC stdc++fs)
endif()
# Add an alias so that the library can be used inside the build tree, e.g. when testing.
add_library(SRL::${PROJECT_NAME} ALIAS ${LIB_NAME})
# This is required so that the exported target has the name Supereight2 and not supereight.
set_target_properties(${LIB_NAME} PROPERTIES EXPORT_NAME ${PROJECT_NAME})

# Compile the tests
if(SE_TEST)
    # enable_testing() must be called in the root CMakeLists.txt.
    enable_testing()
    add_subdirectory(test)
endif()

# Compile the app
if(SE_APP)
    add_subdirectory(app)
endif()

# Install the library
set(PACKAGE_NAME ${PROJECT_NAME})
string(TOUPPER ${PACKAGE_NAME} PACKAGE_NAME_UC)
# The directory where the .cmake files will be installed
include(GNUInstallDirs)
if(WIN32 AND NOT CYGWIN)
    set(INSTALL_CMAKEDIR "${PACKAGE_NAME}/cmake")
elseif(APPLE)
    set(INSTALL_CMAKEDIR "${PACKAGE_NAME}.framework/Resources/CMake")
else()
    set(INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PACKAGE_NAME}")
endif()
# Generate *Config.cmake and *ConfigVersion.cmake files
include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/${PACKAGE_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
    INSTALL_DESTINATION ${INSTALL_CMAKEDIR})
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake
    COMPATIBILITY SameMajorVersion)
# Install files
install(DIRECTORY include/se ${CMAKE_CURRENT_BINARY_DIR}/include/se DESTINATION include PATTERN "*.in" EXCLUDE)
install(TARGETS ${LIB_NAME} EXPORT ${PACKAGE_NAME}-targets DESTINATION lib)
install(EXPORT ${PACKAGE_NAME}-targets
    FILE ${PACKAGE_NAME}Targets.cmake
    NAMESPACE SRL::
    DESTINATION ${INSTALL_CMAKEDIR})
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake
    DESTINATION ${INSTALL_CMAKEDIR})
