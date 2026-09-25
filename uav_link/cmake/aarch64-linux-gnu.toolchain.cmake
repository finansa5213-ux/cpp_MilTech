# Крос-компіляція під Raspberry Pi 5 (ARM64, Debian Trixie).
#
# Збирання на ноутбуці:
#   sudo apt install g++-aarch64-linux-gnu
#   cmake -S . -B build-arm64 \
#         -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.toolchain.cmake \
#         -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-arm64 -j
#
# Збирати можна й просто на малині (`cmake -S . -B build && cmake --build build`) -
# Pi 5 компілює цей проєкт за кілька секунд. Крос-компіляція потрібна тоді,
# коли образ готується наперед і на борту немає компілятора.

set(CMAKE_SYSTEM_NAME      Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Cortex-A76 - ядро Raspberry Pi 5.
set(ARCH_FLAGS "-mcpu=cortex-a76")
set(CMAKE_C_FLAGS_INIT   "${ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${ARCH_FLAGS}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
