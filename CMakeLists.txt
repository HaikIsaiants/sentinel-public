cmake_minimum_required(VERSION 3.25)

project(Sentinel VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

option(BUILD_TESTING "Build tests" ON)

# TODO: CMake presets for the Linux benchmark and day-to-day debug builds

# Local memory checks on Linux:
# add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
# add_link_options(-fsanitize=address,undefined)

find_package(Protobuf REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

include(FetchContent)

set(SENTINEL_BUILD_TESTING ${BUILD_TESTING})
set(BUILD_TESTING OFF)
set(BTCPP_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BTCPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BTCPP_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BTCPP_GROOT_INTERFACE OFF CACHE BOOL "" FORCE)
set(BTCPP_SQLITE_LOGGING OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    behaviortree_cpp
    URL https://github.com/BehaviorTree/BehaviorTree.CPP/archive/3ff6a32ba0497a08519c77a1436e3b81eff1bcd6.tar.gz
    URL_HASH SHA256=84486d12ce7249a611726dd4fbf4a43a9b2f3abf0b00448b6d910a3a54d9a281
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_MakeAvailable(behaviortree_cpp)
set(BUILD_TESTING ${SENTINEL_BUILD_TESTING})

set(SENTINEL_PROTO ${CMAKE_SOURCE_DIR}/proto/sentinel/v1/sentinel.proto)
set(SENTINEL_GENERATED ${CMAKE_BINARY_DIR}/generated)
set(SENTINEL_PYTHON ${CMAKE_BINARY_DIR}/python)
set(SENTINEL_PROTO_SOURCE ${SENTINEL_GENERATED}/sentinel/v1/sentinel.pb.cc)
set(SENTINEL_PROTO_HEADER ${SENTINEL_GENERATED}/sentinel/v1/sentinel.pb.h)
set(SENTINEL_PROTO_PYTHON ${SENTINEL_PYTHON}/sentinel/v1/sentinel_pb2.py)

file(MAKE_DIRECTORY ${SENTINEL_GENERATED} ${SENTINEL_PYTHON})

add_custom_command(
    OUTPUT ${SENTINEL_PROTO_SOURCE} ${SENTINEL_PROTO_HEADER} ${SENTINEL_PROTO_PYTHON}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        --proto_path=${CMAKE_SOURCE_DIR}/proto
        --cpp_out=${SENTINEL_GENERATED}
        --python_out=${SENTINEL_PYTHON}
        ${SENTINEL_PROTO}
    DEPENDS ${SENTINEL_PROTO}
    VERBATIM
)

add_library(sentinel_proto
    ${SENTINEL_PROTO_SOURCE}
    proto/src/framing.cpp
)

target_include_directories(sentinel_proto
    PUBLIC
        ${SENTINEL_GENERATED}
        proto/include
)

target_link_libraries(sentinel_proto PUBLIC protobuf::libprotobuf)

add_library(sentinel_planning
    planning/src/planner.cpp
)

target_include_directories(sentinel_planning PUBLIC planning/include)
target_link_libraries(sentinel_planning PUBLIC sentinel_proto Eigen3::Eigen)

add_library(sentinel_core
    core/src/event_log.cpp
    core/src/hash.cpp
    core/src/network.cpp
    core/src/rng.cpp
    core/src/scenario.cpp
    core/src/simulator.cpp
)

target_include_directories(sentinel_core PUBLIC core/include)
target_link_libraries(sentinel_core PUBLIC sentinel_proto sentinel_planning)

add_executable(sentinel_sim sim/src/main.cpp)
target_link_libraries(sentinel_sim PRIVATE sentinel_core)

add_library(sentinel_agent_logic
    agent/src/allocation.cpp
    agent/src/autonomy.cpp
)
target_include_directories(sentinel_agent_logic PUBLIC agent/include)
target_link_libraries(sentinel_agent_logic PUBLIC sentinel_proto sentinel_planning BT::behaviortree_cpp)

add_executable(sentinel_agent agent/src/main.cpp)
target_link_libraries(sentinel_agent PRIVATE sentinel_agent_logic)

function(sentinel_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

sentinel_warnings(sentinel_proto)
sentinel_warnings(sentinel_planning)
sentinel_warnings(sentinel_core)
sentinel_warnings(sentinel_sim)
sentinel_warnings(sentinel_agent_logic)
sentinel_warnings(sentinel_agent)

if(UNIX)
    add_executable(sentinel_supervisor runner/src/main.cpp)
    target_link_libraries(sentinel_supervisor PRIVATE sentinel_proto)
    sentinel_warnings(sentinel_supervisor)
endif()

include(CTest)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    include(GoogleTest)

    add_executable(sentinel_tests
        tests/event_log_test.cpp
        tests/framing_test.cpp
        tests/network_test.cpp
        tests/planner_test.cpp
        tests/reservation_test.cpp
        tests/rng_test.cpp
        tests/scenario_test.cpp
        tests/simulator_test.cpp
        tests/mission_execution_test.cpp
        tests/distributed_allocation_test.cpp
        tests/failure_recovery_test.cpp
        tests/trace_export_test.cpp
        tests/autonomy_test.cpp
        tests/route_coordination_test.cpp
        tests/allocation_test.cpp
    )

    target_compile_definitions(sentinel_tests PRIVATE SENTINEL_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    target_link_libraries(sentinel_tests PRIVATE sentinel_core sentinel_agent_logic GTest::gtest_main)
    sentinel_warnings(sentinel_tests)
    gtest_discover_tests(sentinel_tests)

    add_test(
        NAME seed_corpus
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/generate_seeds.py --check
    )

    add_test(
        NAME benchmark_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/test_benchmark.py
    )

    add_test(
        NAME scenario_generator_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/test_scenario_generator.py
    )

    add_test(
        NAME agent_smoke
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/test_agent.py
            --agent $<TARGET_FILE:sentinel_agent>
    )

    add_test(
        NAME runner_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/test_runner.py
    )

    if(UNIX)
        add_executable(sentinel_supervisor_fixture tests/supervisor_fixture.cpp)
        sentinel_warnings(sentinel_supervisor_fixture)
        add_test(
            NAME supervisor_tests
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/test_supervisor.py
                --supervisor $<TARGET_FILE:sentinel_supervisor>
                --sim $<TARGET_FILE:sentinel_sim>
                --agent $<TARGET_FILE:sentinel_agent>
                --fixture $<TARGET_FILE:sentinel_supervisor_fixture>
        )
        set_tests_properties(supervisor_tests PROPERTIES
            ENVIRONMENT "PYTHONPATH=${SENTINEL_PYTHON}"
            TIMEOUT 300
        )
    endif()

    add_test(
        NAME benchmark_e2e
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/test_benchmark_e2e.py
            --sim $<TARGET_FILE:sentinel_sim>
            --agent $<TARGET_FILE:sentinel_agent>
    )

    if(UNIX)
        add_test(
            NAME native_benchmark_e2e
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/test_benchmark_e2e.py
                --sim $<TARGET_FILE:sentinel_sim>
                --agent $<TARGET_FILE:sentinel_agent>
                --supervisor $<TARGET_FILE:sentinel_supervisor>
        )
        set_tests_properties(native_benchmark_e2e PROPERTIES
            ENVIRONMENT "PYTHONPATH=${SENTINEL_PYTHON}"
            TIMEOUT 300
        )
    endif()

    function(add_scenario_test name scenario output)
        add_test(
            NAME ${name}
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/run_scenario.py
                --sim $<TARGET_FILE:sentinel_sim>
                --agent $<TARGET_FILE:sentinel_agent>
                --scenario ${CMAKE_SOURCE_DIR}/scenarios/${scenario}.textproto
                --output ${CMAKE_BINARY_DIR}/test-output/${output}
                --repeat 3
                ${ARGN}
        )
    endfunction()

    add_scenario_test(nominal_e2e nominal nominal)
    add_scenario_test(stress_e2e stress stress)
    add_scenario_test(basic_mission_e2e basic_mission basic-mission)
    add_scenario_test(charger_reroute_e2e charger_reroute charger-reroute
        --require-positive replan_count
        --require-positive recharge_ticks
        --require-positive return_ticks
    )
    add_scenario_test(connected_cbba_e2e connected_cbba connected-cbba
        --require-positive communication_messages
        --require-positive delivered_messages
    )
    add_scenario_test(connected_nearest_e2e connected_nearest connected-nearest
        --require-positive communication_messages
        --require-positive delivered_messages
    )
    add_scenario_test(impaired_network_cbba_e2e impaired_network_cbba impaired-network-cbba
        --require-positive communication_messages
        --require-positive delivered_messages
        --require-positive dropped_messages
        --require-positive reordered_messages
    )
    add_scenario_test(impaired_network_nearest_e2e impaired_network_nearest impaired-network-nearest
        --require-positive communication_messages
        --require-positive delivered_messages
        --require-positive dropped_messages
        --require-positive reordered_messages
    )
    add_scenario_test(chokepoint_cbba_e2e chokepoint_cbba chokepoint-cbba
        --require-positive route_conflicts
        --require-positive wait_ticks
    )
    add_scenario_test(chokepoint_nearest_e2e chokepoint_nearest chokepoint-nearest
        --require-positive route_conflicts
        --require-positive wait_ticks
    )
    add_scenario_test(compound_failure_cbba_e2e compound_failure_cbba compound-failure-cbba
        --require-complete-reassignments
        --require-positive maximum_reassignment_delay_ms
        --require-positive replan_count
        --require-positive dropped_messages
    )
    add_scenario_test(compound_failure_nearest_e2e compound_failure_nearest compound-failure-nearest
        --require-complete-reassignments
        --require-positive maximum_reassignment_delay_ms
        --require-positive replan_count
        --require-positive dropped_messages
    )

    set_tests_properties(agent_smoke runner_tests benchmark_tests scenario_generator_tests benchmark_e2e
        nominal_e2e stress_e2e basic_mission_e2e charger_reroute_e2e
        connected_cbba_e2e connected_nearest_e2e impaired_network_cbba_e2e
        impaired_network_nearest_e2e chokepoint_cbba_e2e chokepoint_nearest_e2e
        compound_failure_cbba_e2e compound_failure_nearest_e2e PROPERTIES
        ENVIRONMENT "PYTHONPATH=${SENTINEL_PYTHON}"
        TIMEOUT 120
    )

    set_tests_properties(benchmark_e2e PROPERTIES TIMEOUT 300)
endif()
