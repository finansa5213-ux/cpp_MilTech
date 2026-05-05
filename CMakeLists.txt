cmake_minimum_required(VERSION 3.20)

# Загальний CMake-проєкт для матеріалів блоку 2.
# Кожна homework_* або demos/* директорія підключається окремо.
project(section2 LANGUAGES CXX)

# C++20 - базовий стандарт курсу. EXTENSIONS OFF прибирає GNU-розширення,
# щоб код лишався ближчим до переносимого ISO C++.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# clangd читає compile_commands.json і завдяки цьому бачить ті самі include
# paths, defines і flags, що й реальна CMake-збірка.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Домашні роботи. По мірі появи додаємо кожну окремим add_subdirectory.
add_subdirectory(homework_04)

# Demo-код для заняття 2.4: локальне і віддалене відлагодження через GDB.
add_subdirectory(demos/lesson_2_4/debug_probe)
