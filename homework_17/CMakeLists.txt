# homework_17 — ДЗ Заняття 34: телеметрія дрона по MAVLink 2 / UDP.
# Файл підключається з кореневого CMakeLists через add_subdirectory(homework_17),
# тому тут немає cmake_minimum_required() і project().

# ---------------------------------------------------------------------------
# MAVLink c_library_v2 (header-only).
# Спочатку шукаємо локальний клон у корені репозиторію (як в умові ДЗ),
# інакше тягнемо через FetchContent.
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
# Бібліотека з логікою: те, що покривається тестами.
# ---------------------------------------------------------------------------
add_library(hw17_mavtel STATIC
    src/geo.cpp
    src/udp_socket.cpp
    src/mavlink_link.cpp
    src/drone_sim.cpp
)

target_include_directories(hw17_mavtel PUBLIC include)

# SYSTEM — щоб згенеровані хедери MAVLink не сипали попередженнями під -Wall -Wextra.
target_include_directories(hw17_mavtel SYSTEM PUBLIC "${MAVLINK_DIR}")

target_compile_features(hw17_mavtel PUBLIC cxx_std_17)
target_compile_options(hw17_mavtel PRIVATE -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# Виконуваний файл симулятора.
# ---------------------------------------------------------------------------
add_executable(drone src/main.cpp)
target_link_libraries(drone PRIVATE hw17_mavtel)
target_compile_options(drone PRIVATE -Wall -Wextra -Wpedantic)

find_package(Threads REQUIRED)
target_link_libraries(drone PRIVATE Threads::Threads)

# ---------------------------------------------------------------------------
# Тести (GoogleTest підключається кореневим CMakeLists).
#
# Дві причини, чому тут не проста умова if(BUILD_TESTING AND TARGET GTest::gtest_main):
#  1. BUILD_TESTING визначена лише якщо корінь робить include(CTest).
#     Якщо там тільки enable_testing() — змінної немає, і умова хибна.
#  2. Залежно від способу підключення GoogleTest доступна ціль може називатись
#     GTest::gtest_main (сучасний аліас) або gtest_main (прямий таргет).
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
    set(HW17_BUILD_TESTS OFF)   # користувач явно вимкнув тести
endif()
if(NOT HW17_GTEST_TARGET)
    set(HW17_BUILD_TESTS OFF)
    message(STATUS "homework_17: GoogleTest не знайдено, тести пропущено")
endif()

if(HW17_BUILD_TESTS)
    add_executable(homework_17_tests
        tests/test_geo.cpp
        tests/test_mavlink_link.cpp
    )
    target_link_libraries(homework_17_tests PRIVATE hw17_mavtel ${HW17_GTEST_TARGET})
    target_compile_options(homework_17_tests PRIVATE -Wall -Wextra -Wpedantic)

    include(GoogleTest)
    gtest_discover_tests(homework_17_tests)
endif()
