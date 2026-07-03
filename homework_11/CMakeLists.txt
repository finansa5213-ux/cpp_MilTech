# ============================================================
# homework_11 — Керування дроном по UART і автоматичний скид
# (libgpiod + termios; той самий код у sim і на платі)
# ============================================================
cmake_minimum_required(VERSION 3.20)
project(homework_11 CXX)

# --- libgpiod: без нього ціль пропускається (напр. крос-збірка без sysroot) ---
find_path(GPIOD_INCLUDE_DIR gpiod.h)
find_library(GPIOD_LIBRARY gpiod)
if(NOT GPIOD_INCLUDE_DIR OR NOT GPIOD_LIBRARY)
    message(WARNING
        "homework_11: libgpiod ne znaydeno — tsil propushcheno "
        "(sudo apt install libgpiod-dev)")
    return()
endif()

set(TARGET_NAME homework_11)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

option(USE_TSAN "Build with ThreadSanitizer" OFF)

set(SOURCES
    src/main.cpp
    src/MissionProcessor.cpp
    src/DroneController.cpp
    src/io/UartLink.cpp
    src/io/GpioSignals.cpp
    src/solvers/AnalyticalSolver.cpp
)

set(HEADERS
    include/drone_link.h
    include/Types.h
    include/LinkState.h
    include/MissionProcessor.h
    include/DroneController.h
    include/io/UartLink.h
    include/io/GpioSignals.h
    include/interfaces/IBallisticSolver.h
    include/solvers/AnalyticalSolver.h
)

add_executable(${TARGET_NAME} ${SOURCES} ${HEADERS})

target_include_directories(${TARGET_NAME}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)
set_target_properties(${TARGET_NAME} PROPERTIES
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_compile_options(${TARGET_NAME} PRIVATE -Wall -Wextra -Wpedantic)

# --- libgpiod (API v1): sudo apt install libgpiod-dev ---
find_path(GPIOD_INCLUDE_DIR gpiod.h)
find_library(GPIOD_LIBRARY gpiod)
target_include_directories(${TARGET_NAME} PRIVATE ${GPIOD_INCLUDE_DIR})

# v1 має gpiod_chip_open_by_name; якщо його нема — це API v2 (Ubuntu 24.04+)
include(CheckSymbolExists)
set(CMAKE_REQUIRED_INCLUDES  ${GPIOD_INCLUDE_DIR})
set(CMAKE_REQUIRED_LIBRARIES ${GPIOD_LIBRARY})
check_symbol_exists(gpiod_chip_open_by_name gpiod.h HW11_GPIOD_V1)
if(NOT HW11_GPIOD_V1)
    message(STATUS "libgpiod API v2 vyyavleno -> HW11_GPIOD_V2")
    target_compile_definitions(${TARGET_NAME} PRIVATE HW11_GPIOD_V2)
endif()

find_package(Threads REQUIRED)
target_link_libraries(${TARGET_NAME}
    PRIVATE ${GPIOD_LIBRARY} Threads::Threads
)

if(USE_TSAN)
    target_compile_options(${TARGET_NAME} PRIVATE -fsanitize=thread -g -O1)
    target_link_options(${TARGET_NAME} PRIVATE -fsanitize=thread)
endif()
