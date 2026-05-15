# Library with frame reading, parsing and summary calculation.
add_library(telemetry
    src/telemetry.cpp
)

# Public include — visible both to the library itself and to anything
# that links against it (telemetry_check below).
target_include_directories(telemetry
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Executable: thin wrapper that reads argv[1] and prints the summary.
add_executable(telemetry_check
    src/main.cpp
)

target_link_libraries(telemetry_check
    PRIVATE
        telemetry
)