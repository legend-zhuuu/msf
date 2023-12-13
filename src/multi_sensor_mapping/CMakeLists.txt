# cmake_minimum_required(VERSION 3.13)
cmake_minimum_required(VERSION 3.6)

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(multi_sensor_mapping VERSION 1.1.24 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wno-comment -Wno-deprecated-declarations -g -ggdb")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O4 -s -Wall -Wno-sign-compare -Wno-unused -Wno-comment -Wno-deprecated-declarations -g -pthread")

set(USE_OWNER_GPS 0)
set(USE_LICENSE 0)

#========================
#  Path Setup
#========================
set(INSTALL_DRIVER_DIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_PROJECT_NAME}/include)
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_PREFIX}/lib/cmake)
set(MSL_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/src)
set(DRIVER_CMAKE_ROOT ${CMAKE_CURRENT_LIST_DIR}/cmake)

option(COMPILE_TESTS "Build multi_sensor_mapping tests" ON)

configure_file(
    ${PROJECT_SOURCE_DIR}/config.h.in
    ${PROJECT_SOURCE_DIR}/include/config.h
)

configure_file(
    ${PROJECT_SOURCE_DIR}/version.h.in
    ${PROJECT_SOURCE_DIR}/include/version.h
    @ONLY
)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  pcl_ros
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  rosbag
  rviz
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Glog)
find_package(Ceres  REQUIRED)
find_package(Boost REQUIRED)
find_package(octomap REQUIRED)
find_package(rs_driver REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(yaml-cpp REQUIRED)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(zipper REQUIRED zipper)

message(STATUS "Found ceres headers in: ${CERES_INCLUDE_DIRS}")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_message_files(
  DIRECTORY msg
  FILES
  backend_panel_cmd.msg
  adjust_cloud_panel_cmd.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
)

# include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${rs_driver_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS}
)

# add subdirectory
add_subdirectory(external/ndt-omp)

get_property(NDT_OMP_INCLUDE_DIR GLOBAL PROPERTY NDT_OMP_INCLUDE_DIRS)
message(STATUS "NDT_OMP_INCLUDE_DIRS:" ${NDT_OMP_INCLUDE_DIR})
include_directories(${NDT_OMP_INCLUDE_DIR})

# add_subdirectory(external/ufomap)

# get_property(UFOMAP_INCLUDE_DIR GLOBAL PROPERTY UFOMAP_INCLUDE_DIRS)
# message(STATUS "UFOMAP_INCLUDE_DIRS:" ${UFOMAP_INCLUDE_DIR})
# include_directories(${UFOMAP_INCLUDE_DIR})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "Host system processor is aarch64")
  link_directories(lib/aarch64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
	message(STATUS "Host system processor is x86_64")
  link_directories(lib/x86_64)
endif()

SET(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/../install" CACHE STRING "The path to use for make install" FORCE)
SET(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE) 

################
## Rviz Panel ##
################

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(MOC_FILES
    include/panel/adjust_cloud_panel.h
    include/panel/region_selection_tool.h
    include/panel/backend_panel.h
)

set(SOURCE_FILES
  src/panel/adjust_cloud_panel.cpp
  src/panel/region_selection_tool.cpp
  src/panel/backend_panel.cc
  ${MOC_FILES}
        ../aliengo_description/src/tf_robot_pose.cpp
)

add_library(adjust_cloud_panel ${SOURCE_FILES})
target_link_libraries(adjust_cloud_panel ${QT_LIBRARIES} ${catkin_LIBRARIES})


#============================
#  Build Lib, App, Tests
#============================
# Find src files 
file(GLOB_RECURSE
    utils_src_files
    ${PROJECT_SOURCE_DIR}/src/multi_sensor_mapping/utils/*.cc
    ${PROJECT_SOURCE_DIR}/src/multi_sensor_mapping/param/*.cc
    ${PROJECT_SOURCE_DIR}/src/multi_sensor_mapping/visualizer/*.cc
)

add_library (
    msm_utils  
    SHARED  
    ${utils_src_files}
)
target_link_libraries(msm_utils
    ${catkin_LIBRARIES}    
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${rs_driver_LIBRARIES}
)

file(GLOB_RECURSE
    frontend_src_files
    ${PROJECT_SOURCE_DIR}/src/multi_sensor_mapping/frontend/*.cc
    ${PROJECT_SOURCE_DIR}/src/multi_sensor_mapping/spline/*.cc
    ${PROJECT_SOURCE_DIR}/src/multi_sensor_mapping/factor/*.cc
)

add_library (
    msm_frontend 
    SHARED  
    ${frontend_src_files}
)
target_link_libraries(msm_frontend
    ${PCL_LIBRARIES}
    ${GeographicLib_LIBRARIES}
    ${OpenCV_LIBRARIES}
    msm_utils
    ndt_omp
    ceres
    gtsam
)

file(GLOB_RECURSE
    core_src_files
    ${PROJECT_SOURCE_DIR}/src/multi_sensor_mapping/core/*.cc
    ${PROJECT_SOURCE_DIR}/src/multi_sensor_mapping/map/*.cc
    ${PROJECT_SOURCE_DIR}/src/multi_sensor_mapping/backend/*.cc
)

add_library (
    msm_core 
    SHARED  
    ${core_src_files}
)
target_link_libraries(msm_core
    msm_utils
    msm_frontend
    # UFO::Map
)

#============================
#  Build SDK
#============================
file(GLOB_RECURSE
    sdk_src_files
    ${PROJECT_SOURCE_DIR}/src/msm_sdk/*.cc
)

add_library (
  msm_sdk
  SHARED
  ${sdk_src_files}
)
target_link_libraries(msm_sdk
  msm_frontend
  msm_utils
  msm_core
)

#############
##  Nodes  ##
#############

add_executable(lidar_imu_mapping_node ${PROJECT_SOURCE_DIR}/app/lidar_imu_mapping_node.cc)
target_link_libraries(lidar_imu_mapping_node ${catkin_LIBRARIES} msm_core )

add_executable(pose_graph_node ${PROJECT_SOURCE_DIR}/app/pose_graph_node.cc)
target_link_libraries(pose_graph_node ${catkin_LIBRARIES} msm_core )

add_executable(lidar_imu_frontend_node ${PROJECT_SOURCE_DIR}/app/lidar_imu_frontend_node.cc)
target_link_libraries(lidar_imu_frontend_node ${catkin_LIBRARIES} msm_core )

add_executable(lidar_imu_odometry_node ${PROJECT_SOURCE_DIR}/app/lidar_imu_odometry_node.cc)
target_link_libraries(lidar_imu_odometry_node ${catkin_LIBRARIES} msm_core )

add_executable(low_cost_mapping_node ${PROJECT_SOURCE_DIR}/app/low_cost_mapping_node.cc)
target_link_libraries(low_cost_mapping_node ${catkin_LIBRARIES} msm_core )

add_executable(low_cost_location_node ${PROJECT_SOURCE_DIR}/app/low_cost_location_node.cc)
target_link_libraries(low_cost_location_node ${catkin_LIBRARIES} msm_core )

add_executable(incremental_pose_graph_node ${PROJECT_SOURCE_DIR}/app/incremental_pose_graph_node.cc)
target_link_libraries(incremental_pose_graph_node ${catkin_LIBRARIES} msm_core )

#############
##  Tools  ##
#############
add_executable(matrix2xyzrpy src/tools/matrix2xyzrpy.cc)
target_link_libraries(matrix2xyzrpy ${catkin_LIBRARIES} msm_utils)

add_executable(feature_extractor_viewer src/tools/feature_extractor_viewer.cc)
target_link_libraries(feature_extractor_viewer ${catkin_LIBRARIES} msm_core)

add_executable(ground_segmentation_viewer src/tools/ground_segmentation_viewer.cc)
target_link_libraries(ground_segmentation_viewer ${catkin_LIBRARIES} msm_core)

add_executable(reflection_column_viewer src/tools/reflection_column_viewer.cc)
target_link_libraries(reflection_column_viewer ${catkin_LIBRARIES} msm_core)

add_executable(cloud_viewer src/tools/cloud_viewer.cc)
target_link_libraries(cloud_viewer ${catkin_LIBRARIES} msm_core)

add_executable(export_cloud_tool src/tools/export_cloud_tool.cc)
target_link_libraries(export_cloud_tool ${catkin_LIBRARIES} msm_core)

add_executable(map_conflation_tool src/tools/map_conflation_tool.cc)
target_link_libraries(map_conflation_tool ${catkin_LIBRARIES} msm_core)

add_executable(bag_rewrite_tool src/tools/bag_rewrite_tool.cc)
target_link_libraries(bag_rewrite_tool ${catkin_LIBRARIES} msm_core)

# add_executable(surfel_map_generator src/tools/surfel_map_generator.cc)
# target_link_libraries(surfel_map_generator ${catkin_LIBRARIES} msm_core)

#############
##  Test  ##
#############

if(${COMPILE_TESTS})
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/test)
endif(${COMPILE_TESTS})

############
## INSTAL ##
############
# install(TARGETS
#     imu_initializer_test
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# install(TARGETS
#   msm_utils msm_frontend msm_sdk ndt_omp
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(TARGETS
  msm_utils msm_frontend msm_core msm_sdk 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/msm_sdk/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)