# ============================================================
# homework_15 — Постинг результатів симуляції на сервер курсу.
#
# Підпроєкт кореневого CMakeLists (без cmake_minimum_required
# і project() — вони задані в корені, як і в решті ДЗ).
# ============================================================
set(TARGET_NAME homework_15)

set(SOURCES
    src/main.cpp
    src/ResultUploader.cpp
)

set(HEADERS
    include/UploadTypes.h
    include/ResultUploader.h
    include/httplib.h
    include/json.hpp
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

# cpp-httplib у режимі блокуючих сокетів потребує потоків.
find_package(Threads REQUIRED)
target_link_libraries(${TARGET_NAME} PRIVATE Threads::Threads)

if(MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE /W4 /permissive-)
else()
    target_compile_options(${TARGET_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Заголовки сторонніх бібліотек не перевіряємо clang-tidy —
# інакше лог тоне у попередженнях httplib.h і json.hpp.
set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/httplib.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/json.hpp
    PROPERTIES SKIP_LINTING ON
)
