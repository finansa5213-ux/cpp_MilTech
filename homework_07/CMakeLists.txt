# ============================================================
# homework_07 — рефакторинг ДЗ3 у класи з інтерфейсами.
# ============================================================

set(TARGET_NAME homework_07)

set(SOURCES
    src/JsonTargetProvider.cpp
    src/AnalyticalSolver.cpp
    src/FileConfigLoader.cpp
    src/Factory.cpp
    src/MissionProcessor.cpp
    src/main.cpp
)

set(HEADERS
    include/Types.h
    include/ITargetProvider.h
    include/IBallisticSolver.h
    include/IConfigLoader.h
    include/JsonTargetProvider.h
    include/AnalyticalSolver.h
    include/FileConfigLoader.h
    include/Factory.h
    include/MissionProcessor.h
    include/json.hpp
)

add_executable(${TARGET_NAME} ${SOURCES} ${HEADERS})

target_include_directories(${TARGET_NAME}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(${TARGET_NAME} PRIVATE cxx_std_14)

if(MSVC)
    target_compile_definitions(${TARGET_NAME} PRIVATE _CRT_SECURE_NO_WARNINGS)
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