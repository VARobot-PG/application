cmake_minimum_required(VERSION 3.5)
project(influxdb_cpp)

set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_PREFIX_PATH /usr/lib/x86_64-linux-gnu/cmake/ )
set(influxdb_ccp_DIR ../influxdb_cpp)

find_package(libwebsocketpp-dev)

add_subdirectory(lib/fmt)
add_subdirectory(lib/cpprestsdk)

add_library(influxdb_cpp
        src/influxdb_line.h
        src/influxdb_raw_db.cpp
        src/influxdb_raw_db_utf8.cpp
        src/influxdb_simple_api.cpp
        src/influxdb_simple_async_api.cpp
        src/input_sanitizer.cpp)

target_link_libraries(influxdb_cpp cpprest fmt::fmt-header-only)



