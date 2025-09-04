cmake_minimum_required(VERSION 3.9.5)
project(rplidar_fuzz)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

if (CMAKE_CXX_COMPILER)
    # Check if the CXX compiler is afl-clang or afl-clang++
    if (CMAKE_CXX_COMPILER MATCHES "afl-clang-fast")
        message(STATUS "Using AFL Clang compiler")
        # Set flags for AFL
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=fuzzer,address,undefined -ggdb -O2 -fno-omit-frame-pointer")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address,undefined -ggdb -O2")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=address,undefined -ggdb -O2")
    else()
        message(STATUS "Using regular Clang/GCC compiler")
        # Enable fuzzer and address sanitizers for other compilers
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=fuzzer,address,undefined -ggdb -O2")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=fuzzer,address,undefined -ggdb -O2")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=fuzzer,address,undefined -ggdb -O2")
    endif()

    message(STATUS "CXX Flags after: ${CMAKE_CXX_FLAGS}")

    # Include directories for the SDK library
    include_directories(
            ../sdk/include
            ../sdk/src
            ../sdk/src/arch/linux
            ../sdk/src/hal)

    # Create instrumented SDK library
    add_library(rplidar_sdk_instr SHARED
            ../sdk/src/rplidar_driver.cpp
            ../sdk/src/arch/linux/net_serial.cpp
            ../sdk/src/arch/linux/net_socket.cpp
            ../sdk/src/arch/linux/timer.cpp
            ../sdk/src/hal/thread.cpp)

    target_include_directories(rplidar_sdk_instr
            PUBLIC
            ../sdk/include
            ../sdk/src
            PRIVATE
            ../sdk/src/arch/linux
            ../sdk/src/hal)

    # Add the fuzz test executable
    add_executable(harness harness.cc)

    # Link against the instrumented rplidar_sdk library
    target_link_libraries(harness rplidar_sdk_instr)

    # Set the RPATH to find the SDK library
    set_target_properties(harness PROPERTIES INSTALL_RPATH "\$ORIGIN")

    # Set the output directory for the instrumented SDK library
    set_target_properties(rplidar_sdk_instr PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

    # Set the output directory for the fuzz test executable
    set_target_properties(harness PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
else()
    message(FATAL_ERROR "CMAKE_CXX is not set. Please specify the CXX compiler.")
endif ()
