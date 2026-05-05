cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 04. Його можна збирати як
# через root CMakeLists.txt, так і напряму з цієї директорії.
project(ugv_odometry CXX)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Один executable для задачі з UGV odometry.
add_executable(ugv_odometry src/main.cpp)

# Базові compiler warnings допомагають побачити підозрілі місця до runtime.
target_compile_options(ugv_odometry PRIVATE -Wall -Wextra -pedantic)
