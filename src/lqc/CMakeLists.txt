cmake_minimum_required(VERSION 3.16)
project(lqc)

set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O3")
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
endif ()

add_compile_options(-pthread)
set(GAMEPAD_ENABLE_SHARED OFF)
set(GAMEPAD_ENABLE_STATIC OFF)
set(GAMEPAD_ENABLE_INSTALL OFF)
add_subdirectory(thirdparty/libgamepad)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)


add_library(lqc SHARED
    src/interface/agent.cpp
    src/interface/comm.cpp
    src/interface/gamepad.cpp
    src/policy/legacy_policy.cpp
    src/policy/policy_v2.cpp
    src/utils/ev.cpp
    src/utils/math.cpp
    src/utils/imu.cpp
    src/utils/logging.cpp
)

# default values
if (NOT DEFINED LQC_ROBOT)
    set(LQC_ROBOT "aliengo")
endif ()
if (NOT DEFINED LQC_RUNTIME)
    set(LQC_RUNTIME "onnxruntime")
endif ()

if (LQC_ROBOT STREQUAL "aliengo")
    message(STATUS "Building LQC for aliengo")
    add_subdirectory(thirdparty/unitree_legged_sdk/aliengo)
    target_compile_definitions(lqc PUBLIC LQC_ROBOT=LQC_ALIENGO)
elseif (LQC_ROBOT STREQUAL "go1")
    message(STATUS "Building LQC for go1")
    add_subdirectory(thirdparty/unitree_legged_sdk/go1)
    target_compile_definitions(lqc PUBLIC LQC_ROBOT=LQC_GO1)
elseif (LQC_ROBOT STREQUAL "b1")
    message(STATUS "Building LQC for b1")
    add_subdirectory(thirdparty/unitree_legged_sdk/b1)
    target_compile_definitions(lqc PUBLIC LQC_ROBOT=LQC_B1)
else ()
    message(FATAL_ERROR "Unsupported robot `${LQC_ROBOT}`")
endif ()

if (LQC_RUNTIME STREQUAL "onnxruntime")
    message(STATUS "Using onnxruntime for network inference.")
    set(ORT_HOME thirdparty/onnxruntime-x64)
    add_library(runtime INTERFACE)
    find_library(onnxruntime onnxruntime REQUIRED
        PATHS ${ORT_HOME}/lib NO_DEFAULT_PATH
    )
    target_include_directories(runtime INTERFACE ${ORT_HOME}/include)
    target_link_libraries(runtime INTERFACE ${onnxruntime})
    target_sources(lqc PUBLIC src/runtime/ort.cpp)
    install(FILES ${ORT_HOME}/lib/libonnxruntime.so.1.16.1 TYPE LIB)
else ()
    message(FATAL_ERROR "Unsupported runtime `${LQC_RUNTIME}`")
endif ()

if (${LQC_FIX_GETTID})
    target_compile_definitions(lqc PUBLIC LQC_FIX_GETTID)
endif ()

target_include_directories(lqc PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(lqc PUBLIC
    atomic yaml-cpp
    gamepad legged_sdk runtime
)

add_executable(lqc-policy src/main.cpp)
target_link_libraries(lqc-policy lqc)

set_target_properties(lqc PROPERTIES
    INSTALL_RPATH "$ORIGIN"
)
set_target_properties(lqc-policy PROPERTIES
    OUTPUT_NAME "lqc"
    INSTALL_RPATH "$ORIGIN/../lib"
)
install(TARGETS lqc lqc-policy)
