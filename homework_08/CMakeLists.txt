# ============================================================
# homework_08 — структура проекту, STL-контейнери,
# ============================================================

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

# ------------------------------------------------------------
# Копіювання вхідних JSON у каталог, звідки запускається бінарник.
# ------------------------------------------------------------
set(DATA_FILES
    data/config.json
    data/ammo.json
    data/targets.json
)

foreach(DATA_FILE ${DATA_FILES})
    get_filename_component(FILENAME ${DATA_FILE} NAME)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/${DATA_FILE}
        ${CMAKE_CURRENT_BINARY_DIR}/${FILENAME}
        COPYONLY
    )
endforeach()