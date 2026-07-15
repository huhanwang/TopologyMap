cmake_minimum_required(VERSION 3.16)
project(offline_replay LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(offline_replay
    src/main.cpp
    debug/replay_debug_json.cpp
    # debug/global_route_debug/global_route_debug_exporter.cpp
    ../algorithms/boundary_sampling/boundary_sampling_module.cpp
    ../algorithms/frenet/frenet_completed_boundary_builder.cpp
    ../algorithms/frenet/frenet_junction_analyzer.cpp
    ../algorithms/frenet/frenet_lane_region_builder.cpp
    ../algorithms/frenet/frenet_lane_line_completer.cpp
    ../algorithms/frenet/frenet_projector.cpp
    ../algorithms/frenet/frenet_ribbon_builder.cpp
    ../algorithms/frenet/frenet_track_builder.cpp
    ../algorithms/frenet/frenet_types.cpp
    ../algorithms/fused_reference/fused_reference_module.cpp
    ../algorithms/proto_preprocess/proto_preprocess_module.cpp
    ../algorithms/navigation_route/navigation_route_tracker.cpp
    ../algorithms/visual_reference/visual_reference_module.cpp
)

target_include_directories(offline_replay PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/debug
    ${CMAKE_CURRENT_SOURCE_DIR}/debug/global_route_debug
    ${CMAKE_CURRENT_SOURCE_DIR}/../algorithms
    ${CMAKE_CURRENT_SOURCE_DIR}/../algorithms/boundary_sampling
    ${CMAKE_CURRENT_SOURCE_DIR}/../algorithms/frenet
    ${CMAKE_CURRENT_SOURCE_DIR}/../algorithms/fused_reference
    ${CMAKE_CURRENT_SOURCE_DIR}/../algorithms/navigation_route
    ${CMAKE_CURRENT_SOURCE_DIR}/../algorithms/proto_preprocess
    ${CMAKE_CURRENT_SOURCE_DIR}/../algorithms/visual_reference
    ${CMAKE_CURRENT_SOURCE_DIR}/../
    ${CMAKE_CURRENT_SOURCE_DIR}/../../visualization-backend/3rdparty/viz_core
    ${CMAKE_CURRENT_SOURCE_DIR}/../../visualization-backend/build/generated/db4_1_protocol
    ${CMAKE_CURRENT_SOURCE_DIR}/../../dependency/type-protocol/1.0.0_linux_ubuntu_gcc9.4.0/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../dependency/thirdparty/protobuf/3.5.0_linux_ubuntu_gcc9.4.0/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../noah-env/protocol
)

find_package(SQLite3 REQUIRED)

find_library(OFFLINE_REPLAY_PROTOBUF
    NAMES protobuf libprotobuf.so.15.0.0
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../dependency/thirdparty/protobuf/3.5.0_linux_ubuntu_gcc9.4.0/lib
    NO_DEFAULT_PATH
    REQUIRED)
find_library(OFFLINE_REPLAY_ADTYPEPROTOCOL
    NAMES adtypeprotocol libadtypeprotocol.so.1.0.0
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../dependency/type-protocol/1.0.0_linux_ubuntu_gcc9.4.0/lib
    NO_DEFAULT_PATH
    REQUIRED)
find_library(OFFLINE_REPLAY_ADPROTOCOL
    NAMES adprotocol libadprotocol.so.1.0.0
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../dependency/protocol/1.0.0_linux_ubuntu_gcc9.4.0/lib
    NO_DEFAULT_PATH
    REQUIRED)
find_library(OFFLINE_REPLAY_DB4_1_PROTO
    NAMES db4_1_proto libdb4_1_proto.a
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../visualization-backend/build/lib
    NO_DEFAULT_PATH
    REQUIRED)
find_library(OFFLINE_REPLAY_AUTOPROTOCOL
    NAMES autoprotocol libautoprotocol.so.1.0.0
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../noah-env/libs/release
    NO_DEFAULT_PATH
    REQUIRED)
find_library(OFFLINE_REPLAY_ADDART
    NAMES addart libaddart.so.1.0.0
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../dependency/dart/1.0.0_linux_ubuntu_gcc9.4.0/lib
    NO_DEFAULT_PATH
    REQUIRED)
find_library(OFFLINE_REPLAY_ADDARTSHIM
    NAMES addartshim libaddartshim.so.1.0.0
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../dependency/dart/1.0.0_linux_ubuntu_gcc9.4.0/lib
    NO_DEFAULT_PATH
    REQUIRED)
find_library(OFFLINE_REPLAY_ZMQ
    NAMES zmq libzmq.so.5
    PATHS /lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu
    REQUIRED)
find_library(OFFLINE_REPLAY_UUID
    NAMES uuid libuuid.so.1
    PATHS /lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu
    REQUIRED)

target_link_libraries(offline_replay PRIVATE
    SQLite::SQLite3
    ${OFFLINE_REPLAY_DB4_1_PROTO}
    ${OFFLINE_REPLAY_ADPROTOCOL}
    ${OFFLINE_REPLAY_ADTYPEPROTOCOL}
    ${OFFLINE_REPLAY_ADDARTSHIM}
    ${OFFLINE_REPLAY_ADDART}
    ${OFFLINE_REPLAY_AUTOPROTOCOL}
    ${OFFLINE_REPLAY_PROTOBUF}
    ${OFFLINE_REPLAY_ZMQ}
    ${OFFLINE_REPLAY_UUID}
    stdc++fs
)

set_target_properties(offline_replay PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../dependency/protocol/1.0.0_linux_ubuntu_gcc9.4.0/lib;${CMAKE_CURRENT_SOURCE_DIR}/../../dependency/type-protocol/1.0.0_linux_ubuntu_gcc9.4.0/lib;${CMAKE_CURRENT_SOURCE_DIR}/../../dependency/dart/1.0.0_linux_ubuntu_gcc9.4.0/lib;${CMAKE_CURRENT_SOURCE_DIR}/../../dependency/thirdparty/protobuf/3.5.0_linux_ubuntu_gcc9.4.0/lib;${CMAKE_CURRENT_SOURCE_DIR}/../../noah-env/libs/release"
)
