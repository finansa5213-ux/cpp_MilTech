# ============================================================
# homework_08 — STL-контейнери + DroneStateMachine
# ============================================================
cmake_minimum_required(VERSION 3.14)
project(homework_08 CXX)

set(TARGET_NAME homework_08)

set(SOURCES
    src/main.cpp
    src/MissionProcessor.cpp
    src/drone/DroneStateMachine.cpp
    src/solvers/AnalyticalSolver.cpp
    src/providers/JsonTargetProvider.cpp
    src/config/FileConfigLoader.cpp
    src/config/ComponentFactory.cpp
)

set(HEADERS
    include/Types.h
    include/MissionProcessor.h
    include/interfaces/IBallisticSolver.h
    include/interfaces/IConfigLoader.h
    include/interfaces/ITargetProvider.h
    include/drone/DroneStateMachine.h
    include/solvers/AnalyticalSolver.h
    include/providers/JsonTargetProvider.h
    include/config/FileConfigLoader.h
    include/config/ComponentFactory.h
    include/json.hpp
)

add_executable(${TARGET_NAME} ${SOURCES} ${HEADERS})

target_include_directories(${TARGET_NAME}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)

if(MSVC)
    target_compile_definitions(${TARGET_NAME} PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(${TARGET_NAME} PRIVATE /W4)
else()
    target_compile_options(${TARGET_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Копіювання JSON-даних поруч із бінарником
foreach(F config.json ammo.json targets.json)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/data/${F}
        ${CMAKE_CURRENT_BINARY_DIR}/${F}
        COPYONLY
    )
endforeach()