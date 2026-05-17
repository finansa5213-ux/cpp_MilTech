# Бібліотека з доменною логікою балістики.
add_library(ballistics src/ballistics.cpp)
target_include_directories(ballistics PUBLIC include)

# Консольна програма — читає input.txt, пише output.txt.
add_executable(ballistics_cli src/main.cpp)
target_link_libraries(ballistics_cli PRIVATE ballistics)

# Тести через GoogleTest. Завантажуємо його через FetchContent, бо в корені репозиторію він ще не
# підключений.
if(NOT CMAKE_CROSSCOMPILING)
    include(FetchContent)
    FetchContent_Declare(
        googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    # Запобігає конфліктам runtime-бібліотек на Windows; на Linux нешкідливо.
    set(gtest_force_shared_crt
        ON
        CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # Це треба ПІСЛЯ FetchContent_MakeAvailable(googletest), бо саме після нього з'являється модуль
    # GoogleTest.
    include(GoogleTest)

    add_executable(ballistics_tests tests/ballistics_tests.cpp)
    target_link_libraries(ballistics_tests PRIVATE ballistics GTest::gtest_main)
    gtest_discover_tests(ballistics_tests DISCOVERY_MODE PRE_TEST)
endif()
