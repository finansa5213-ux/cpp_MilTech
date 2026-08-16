# ============================================================
# homework_17 — ДЗ Заняття 34: телеметрія MAVLink 2 / UDP.
#
# Розширює ДЗ11: той самий автопілот (UART від чекера ДЗ11 + GPIO),
# але кадр телеметрії паралельно йде в MAVLink на 127.0.0.1:14550.
# Модулі ДЗ11 (солвер, контролер, UART, GPIO) підключаються з
# ../homework_11 без копіювання — ДЗ11 лишається незмінним.
#
# Файл підключається через add_subdirectory(homework_17),
# тому тут немає cmake_minimum_required() і project().
# ============================================================

set(HW11_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../homework_11")

if(NOT EXISTS "${HW11_DIR}/include/drone_link.h")
    message(WARNING "homework_17: ne znaydeno homework_11 — tsil propushcheno")
    return()
endif()

# ---------------------------------------------------------------------------
# libgpiod — як у ДЗ11: без нього ціль пропускається.
# ---------------------------------------------------------------------------
find_path(GPIOD_INCLUDE_DIR gpiod.h)
find_library(GPIOD_LIBRARY gpiod)
if(NOT GPIOD_INCLUDE_DIR OR NOT GPIOD_LIBRARY)
    message(WARNING
        "homework_17: libgpiod ne znaydeno — tsil propushcheno "
        "(sudo apt install libgpiod-dev)")
    return()
endif()

# ---------------------------------------------------------------------------
# MAVLink c_library_v2 (header-only).
# ---------------------------------------------------------------------------
set(MAVLINK_DIR "${CMAKE_SOURCE_DIR}/c_library_v2" CACHE PATH
    "Каталог з mavlink/c_library_v2")

if(NOT EXISTS "${MAVLINK_DIR}/common/mavlink.h")
    include(FetchContent)
    FetchContent_Declare(
        mavlink_c_library_v2
        GIT_REPOSITORY https://github.com/mavlink/c_library_v2.git
        GIT_TAG        master
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(mavlink_c_library_v2)
    set(MAVLINK_DIR "${mavlink_c_library_v2_SOURCE_DIR}")
endif()

message(STATUS "MAVLink headers: ${MAVLINK_DIR}")

# ---------------------------------------------------------------------------
# Бібліотека MAVLink-частини: те, що покривається юніт-тестами.
# Свідомо не залежить ні від UART, ні від GPIO.
# ---------------------------------------------------------------------------
add_library(hw17_mavtel STATIC
    src/geo.cpp
    src/udp_socket.cpp
    src/mavlink_link.cpp
    src/dz11_bridge.cpp
)

target_include_directories(hw17_mavtel PUBLIC
    include
    "${HW11_DIR}/include"   # drone_link.h — протокол ДЗ11
)

# SYSTEM — щоб згенеровані хедери MAVLink не сипали попередженнями під -Wall.
target_include_directories(hw17_mavtel SYSTEM PUBLIC "${MAVLINK_DIR}")

target_compile_features(hw17_mavtel PUBLIC cxx_std_17)
target_compile_options(hw17_mavtel PRIVATE -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# Виконуваний файл: автопілот ДЗ11 + MAVLink.
# Джерела ДЗ11 беруться напряму — дублювання коду немає.
# ---------------------------------------------------------------------------
add_executable(drone
    src/main.cpp
    src/MavMissionProcessor.cpp
    "${HW11_DIR}/src/DroneController.cpp"
    "${HW11_DIR}/src/io/UartLink.cpp"
    "${HW11_DIR}/src/io/GpioSignals.cpp"
    "${HW11_DIR}/src/solvers/AnalyticalSolver.cpp"
)

target_link_libraries(drone PRIVATE hw17_mavtel)
target_compile_options(drone PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(drone PRIVATE ${GPIOD_INCLUDE_DIR})

# libgpiod v1 має gpiod_chip_open_by_name; якщо його немає — це API v2
# (Ubuntu 24.04+). Логіка та сама, що в ДЗ11.
include(CheckSymbolExists)
set(CMAKE_REQUIRED_INCLUDES  ${GPIOD_INCLUDE_DIR})
set(CMAKE_REQUIRED_LIBRARIES ${GPIOD_LIBRARY})
check_symbol_exists(gpiod_chip_open_by_name gpiod.h HW17_GPIOD_V1)
if(NOT HW17_GPIOD_V1)
    message(STATUS "homework_17: libgpiod API v2 -> HW11_GPIOD_V2")
    target_compile_definitions(drone PRIVATE HW11_GPIOD_V2)
endif()

find_package(Threads REQUIRED)
target_link_libraries(drone PRIVATE ${GPIOD_LIBRARY} Threads::Threads)

# ---------------------------------------------------------------------------
# Тести.
#
# Умова навмисно не проста if(BUILD_TESTING AND TARGET GTest::gtest_main):
#  1. BUILD_TESTING визначена лише якщо корінь робить include(CTest);
#     у цьому репозиторії там тільки enable_testing().
#  2. Ціль GoogleTest може називатись GTest::gtest_main або gtest_main.
# ---------------------------------------------------------------------------
if(TARGET GTest::gtest_main)
    set(HW17_GTEST_TARGET GTest::gtest_main)
elseif(TARGET gtest_main)
    set(HW17_GTEST_TARGET gtest_main)
else()
    set(HW17_GTEST_TARGET "")
endif()

set(HW17_BUILD_TESTS ON)
if(DEFINED BUILD_TESTING AND NOT BUILD_TESTING)
    set(HW17_BUILD_TESTS OFF)
endif()
if(NOT HW17_GTEST_TARGET)
    set(HW17_BUILD_TESTS OFF)
    message(STATUS "homework_17: GoogleTest ne znaydeno — testy propushcheno")
endif()

if(HW17_BUILD_TESTS)
    add_executable(homework_17_tests
        tests/test_geo.cpp
        tests/test_mavlink_link.cpp
        tests/test_dz11_bridge.cpp
    )
    target_link_libraries(homework_17_tests PRIVATE hw17_mavtel ${HW17_GTEST_TARGET})
    target_compile_options(homework_17_tests PRIVATE -Wall -Wextra -Wpedantic)

    include(GoogleTest)
    gtest_discover_tests(homework_17_tests)
endif()
