cmake_minimum_required(VERSION 3.20)

# Цей CMakeLists можна збирати двома способами:
# - як частину кореневого репозиторію через add_subdirectory();
# - окремо на цільовому пристрої, якщо скопійовано лише цю demo-директорію.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(debug_probe LANGUAGES CXX)

  # Standalone-збірка використовує той самий C++20, що й основний курс.
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

add_executable(debug_probe src/main.cpp)

# Warning flags тримаємо на target-рівні, щоб не впливати на інші приклади.
target_compile_options(
  debug_probe
  PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>)
