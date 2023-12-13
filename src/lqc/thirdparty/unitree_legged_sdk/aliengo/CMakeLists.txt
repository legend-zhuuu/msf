cmake_minimum_required(VERSION 3.16)
project(aliengo_sdk)

message(STATUS "System processor: ${CMAKE_SYSTEM_PROCESSOR}")
if ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
    set(ARCH amd64)
endif ()
if ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
    set(ARCH arm64)
endif ()

add_library(legged_sdk INTERFACE)
target_include_directories(legged_sdk INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

find_library(unitree_legged_sdk unitree_legged_sdk REQUIRED
    PATHS ${CMAKE_CURRENT_LIST_DIR}/lib/${ARCH} NO_DEFAULT_PATH
)

install(FILES ${unitree_legged_sdk} TYPE LIB)
target_link_libraries(legged_sdk INTERFACE
    ${unitree_legged_sdk} lcm
)
