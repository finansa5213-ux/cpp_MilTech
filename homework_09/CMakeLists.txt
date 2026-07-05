# ============================================================
# homework_09 — Ballistic Mission Processor
# Інтерфейси + розумні вказівники + State + фабрика компонентів
# ============================================================
cmake_minimum_required(VERSION 3.14)
project(homework_09 CXX)

set(TARGET_NAME homework_09)

# --- Тип збірки за замовчуванням ---
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# ------------------------------------------------------------
# Джерела та заголовки
# ------------------------------------------------------------
set(SOURCES
    src/main.cpp
    src/MissionProcessor.cpp
    src/drone/DroneStateMachine.cpp
    src/solvers/AnalyticalSolver.cpp
    src/solvers/TableSolver.cpp
    src/providers/JsonTargetProvider.cpp
    src/config/FileConfigLoader.cpp
    src/config/ComponentFactory.cpp
)

set(HEADERS
    include/Types.h
    include/MissionProcessor.h
    include/json.hpp
    include/interfaces/IBallisticSolver.h
    include/interfaces/IConfigLoader.h
    include/interfaces/ITargetProvider.h
    include/drone/DroneStateMachine.h
    include/solvers/AnalyticalSolver.h
    include/solvers/TableSolver.h
    include/providers/JsonTargetProvider.h
    include/config/FileConfigLoader.h
    include/config/ComponentFactory.h
)

add_executable(${TARGET_NAME} ${SOURCES} ${HEADERS})

# ------------------------------------------------------------
# Налаштування компіляції
# ------------------------------------------------------------
target_include_directories(${TARGET_NAME}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)

set_target_properties(${TARGET_NAME} PROPERTIES
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

if(MSVC)
    target_compile_definitions(${TARGET_NAME} PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(${TARGET_NAME} PRIVATE /W4 /permissive-)
else()
    target_compile_options(${TARGET_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------
# Копіювання даних поруч із бінарником
# ------------------------------------------------------------
set(DATA_FILES
    config.json
    ammo.json
    targets.json
    ballistic_table.txt
)

foreach(F IN LISTS DATA_FILES)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/data/${F}
        ${CMAKE_CURRENT_BINARY_DIR}/${F}
        COPYONLY
    )
endforeach()
