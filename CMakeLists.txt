cmake_minimum_required(VERSION 3.4)
project(camera_calibration)

# Set version
set(CAMERA_CALIBRATION_MAJOR_VERSION 1)
set(CAMERA_CALIBRATION_MINOR_VERSION 1)
set(CAMERA_CALIBRATION_VERSION ${CAMERA_CALIBRATION_MAJOR_VERSION}.${CAMERA_CALIBRATION_MINOR_VERSION})

# Global configurations
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_DEBUG_POSTFIX "_d")
if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
	 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOMINMAX")
endif()
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Build components
# ===================================================
option(BUILD_DOCS "Build documentation using Doxygen" ON)

# Find dependencies
# ===================================================

# Boost
option(WITH_BOOST_STATIC "Boost static libraries" ON)
set(Boost_USE_STATIC_LIBS ${WITH_BOOST_STATIC})
set(BOOST_ALL_DYN_LINK NOT ${WITH_BOOST_STATIC})
if(WIN32)
	if(${WITH_BOOST_STATIC})
		add_definitions(-DBOOST_ALL_NO_LIB)
	else()
		add_definitions(-DBOOST_ALL_DYN_LINK)
	endif()
endif()
find_package(Boost REQUIRED filesystem program_options timer)

# OpenCV
find_package(OpenCV REQUIRED highgui imgproc imgcodecs calib3d)
if(WITH_OPENCV_CONTRIB)
	find_package(OpenCV COMPONENTS face)
endif()

# vsal
find_package(vsal REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# Ceres
find_package(Ceres REQUIRED)

# Docs
if(BUILD_DOCS)
	find_package(Doxygen)
	if(NOT DOXYGEN_FOUND)
		message(STATUS "The documentation won't be build.")
	endif()
endif()

# Add subdirectories
# ===================================================
add_subdirectory(cauldron)
add_subdirectory(sparse_graph)
add_subdirectory(camera_models)
add_subdirectory(camera_calibration)
add_subdirectory(mono_camera_calibration)
add_subdirectory(rectification)
add_subdirectory(rectify)

# Docs
if(BUILD_DOCS AND DOXYGEN_FOUND)
	add_subdirectory(doc)
endif()

# Export configuration
# ===================================================

# Add all targets to the build-tree export set
export(TARGETS ${CC_TARGETS}
  FILE "${PROJECT_BINARY_DIR}/camera_calibration-targets.cmake")
  
# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE camera_calibration)
	 
# Create config files
configure_file(cmake/camera_calibration-config.cmake.in
  "${PROJECT_BINARY_DIR}/camera_calibration-config.cmake" @ONLY)
configure_file(cmake/camera_calibration-config-version.cmake.in
  "${PROJECT_BINARY_DIR}/camera_calibration-config-version.cmake" @ONLY)
  
# Install config files
install(FILES
  "${PROJECT_BINARY_DIR}/camera_calibration-config.cmake"
  "${PROJECT_BINARY_DIR}/camera_calibration-config-version.cmake"
  DESTINATION "cmake" COMPONENT dev)
  
# Install the export set for use with the install-tree
install(EXPORT camera_calibration-targets DESTINATION cmake COMPONENT dev)
