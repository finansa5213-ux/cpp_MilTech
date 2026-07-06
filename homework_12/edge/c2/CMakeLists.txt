cmake_minimum_required(VERSION 3.20)
project(c2_service CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(MAVSDK REQUIRED)

include(FetchContent)

FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(JSON_Install    OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(nlohmann_json)

add_executable(c2_service
    src/main.cpp
    src/fc_link.cpp
    src/c2_controller.cpp
)

target_include_directories(c2_service PRIVATE ${CMAKE_SOURCE_DIR}/src)

target_link_libraries(c2_service PRIVATE
    MAVSDK::mavsdk
    nlohmann_json::nlohmann_json
)

target_compile_options(c2_service PRIVATE -Wall -Wextra)
