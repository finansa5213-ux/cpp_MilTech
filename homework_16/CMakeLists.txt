# Підпроєкт ДЗ16 (Заняття 32) — I²C-датчик MPU-6050.
# Кореневий CMakeLists підключає його через add_subdirectory(homework_16),
# тому тут НЕ має бути cmake_minimum_required() і project().

# --- Бібліотека з логікою (без main) -----------------------------------------
add_library(mpu6050_core STATIC
    src/I2cError.cpp
    src/LinuxI2cBus.cpp
    src/Mpu6050.cpp
)

target_include_directories(mpu6050_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(mpu6050_core PUBLIC cxx_std_17)
target_compile_options(mpu6050_core PRIVATE -Wall -Wextra -Wpedantic)

# --- Виконуваний файл ---------------------------------------------------------
add_executable(homework_16 src/main.cpp)
target_link_libraries(homework_16 PRIVATE mpu6050_core)
target_compile_options(homework_16 PRIVATE -Wall -Wextra -Wpedantic)

# --- Тести --------------------------------------------------------------------
# GoogleTest може вже бути в проєкті (кореневий CMakeLists тягне його через
# FetchContent для попередніх ДЗ), бути встановленим у системі, або не бути
# зовсім. Перевіряємо в цьому порядку й ПОВТОРНО НЕ ЗАВАНТАЖУЄМО — інакше
# FetchContent конфліктує з уже наповненим _deps/googletest.
set(HW16_GTEST "")

if(TARGET GTest::gtest_main)
    set(HW16_GTEST GTest::gtest_main)
elseif(TARGET gtest_main)
    # Старіші версії googletest не створюють алias GTest::.
    set(HW16_GTEST gtest_main)
else()
    find_package(GTest QUIET)
    if(TARGET GTest::gtest_main)
        set(HW16_GTEST GTest::gtest_main)
    else()
        include(FetchContent)
        FetchContent_Declare(googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v1.14.0
        )
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(googletest)
        if(TARGET GTest::gtest_main)
            set(HW16_GTEST GTest::gtest_main)
        elseif(TARGET gtest_main)
            set(HW16_GTEST gtest_main)
        endif()
    endif()
endif()

if(HW16_GTEST)
    add_executable(mpu6050_tests tests/mpu6050_tests.cpp)
    target_link_libraries(mpu6050_tests PRIVATE mpu6050_core ${HW16_GTEST})
    target_compile_options(mpu6050_tests PRIVATE -Wall -Wextra -Wpedantic)

    include(GoogleTest)
    gtest_discover_tests(mpu6050_tests)
else()
    message(WARNING "GoogleTest не знайдено — ціль mpu6050_tests пропущено")
endif()
