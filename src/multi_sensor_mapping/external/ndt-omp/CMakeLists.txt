cmake_minimum_required(VERSION 2.8.3)
project(ndt_omp)

# -mavx causes a lot of errors!!
message(STATUS "SYSTEM ARCH : " ${CMAKE_HOST_SYSTEM_PROCESSOR})
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
 add_definitions(-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
 set(CMAKE_CXX_FLAGS "-std=c++14 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
else()
  set(CMAKE_CXX_FLAGS "-std=c++14 -fopenmp")
endif()



# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

#=======================================
# Compile setup (ORIGINAL, CATKIN)
#=======================================
set(COMPILE_METHOD ORIGINAL)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


set_property(GLOBAL PROPERTY NDT_OMP_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)


###################################
## catkin specific configuration ##
###################################

if(${COMPILE_METHOD} STREQUAL "CATKIN")
  find_package(roscpp 1.12 QUIET)
  if(roscpp_FOUND)
    message(=============================================================)
    message("-- ROS Found, compile the project with catkin make!")
    message(=============================================================)
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      pcl_ros
    )

    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ndt_omp
    )
    include_directories(${catkin_INCLUDE_DIRS})

    add_definitions(-DROS_FOUND)
  else(roscpp_FOUND)
    message(=============================================================)
    message("-- ROS Not Found, Ros Support is turned Off!")
    message(=============================================================)
  endif(roscpp_FOUND)

elseif (${COMPILE_METHOD} STREQUAL "ORIGINAL")  
  message(=============================================================)
  message("-- Compile the project with cmake !")
  message(=============================================================)


endif()

###########
## Build ##
###########
include_directories(include)

add_library(ndt_omp
  SHARED
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

add_executable(cloud_align_benchmark
  apps/cloud_align_benchmark.cpp
)
add_dependencies(cloud_align_benchmark
  ndt_omp
)
target_link_libraries(cloud_align_benchmark
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
)




############
## INSTAL ##
############

if(${COMPILE_METHOD} STREQUAL "CATKIN")
  if(roscpp_FOUND)
    install(
      TARGETS
        ndt_omp
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )

    # install headers
    install(DIRECTORY include/pclomp
            DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    endif(roscpp_FOUND)
elseif(${COMPILE_METHOD} STREQUAL "ORIGINAL")
  install(
    TARGETS
      ndt_omp
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

endif(${COMPILE_METHOD} STREQUAL "CATKIN")
