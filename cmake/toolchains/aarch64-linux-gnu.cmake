# Toolchain для cross-compilation з Linux x86_64 devcontainer-а під ARM64 Linux.
# Підходить для Raspberry Pi 4, Radxa ROCK 5B+ і Jetson з 64-bit Linux userspace.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Компілятори з Debian/Ubuntu пакета g++-aarch64-linux-gnu.
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Пошук headers/libraries йде у target sysroot, а host-програм - на host-і.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
