enable_testing()

# Use multithreaded Boost libraries
set(Boost_USE_MULTITHREADED ON)

# Use static boost libraries so the dynamic library 
# can run anywhere
set(Boost_USE_STATIC_LIBS ON)

# Find the necessary boost components on the system.
find_package (Boost 1.66 REQUIRED COMPONENTS thread system regex date_time chrono unit_test_framework)

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
add_executable(test_h5_transport test_h5_transport.cpp)

target_link_libraries(test_uart_boost PRIVATE pc_ble_driver_static_sd_api_v3 ${Boost_LIBRARIES})
target_link_libraries(test_h5_transport PRIVATE pc_ble_driver_static_sd_api_v3 ${Boost_LIBRARIES})

add_test(NAME uart_boost COMMAND test_uart_boost)
add_test(NAME h5_transport COMMAND test_h5_transport)

