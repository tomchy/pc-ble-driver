cmake_minimum_required(VERSION 3.3)
project(transport_tests)
enable_testing()

# Compiler specific
if(MSVC)
    include(${CMAKE_CURRENT_LIST_DIR}/../cmake/msvc.cmake)
elseif(APPLE)
    include(${CMAKE_CURRENT_LIST_DIR}/../cmake/apple.cmake)
else()
    # Linux
    include(${CMAKE_CURRENT_LIST_DIR}/../cmake/gcc.cmake)
endif()

# Use multithreaded Boost libraries
set(Boost_USE_MULTITHREADED ON)

# Use static boost libraries so the dynamic library 
# can run anywhere
set(Boost_USE_STATIC_LIBS ON)

# Find the necessary boost components on the system.
# Minimum version required is 1.54.0
find_package ( Boost 1.65.1 REQUIRED COMPONENTS thread system regex date_time chrono unit_test_framework)

link_directories (
    ../build
)

include_directories (
    ../include/common/sdk_compat
    ../include/common
    ../include/common/internal/transport
    ../src/sd_api_v3/sdk/components/softdevice/s132/headers
     SYSTEM ${Boost_INCLUDE_DIRS}
)

if(MSVC)
    add_definitions(-DPC_BLE_DRIVER_STATIC)
endif()

# Build executable
add_executable(test_uart_boost test_uart_boost.cpp)

target_link_libraries(test_uart_boost PRIVATE pc_ble_driver_static_sd_api_v3 ${Boost_LIBRARIES})

add_test(NAME uart_boost COMMAND test_uart_boost)
