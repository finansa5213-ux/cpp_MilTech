cmake_minimum_required(VERSION 3.16)
project(uav_link LANGUAGES CXX
        DESCRIPTION "Бортовий маршрутизатор каналів керування БПЛА")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor)

# Ядро: усе, крім точки входу. Окремою бібліотекою, щоб тести збиралися
# з тим самим кодом, що й програма, а не з його копією.
add_library(uav_core STATIC
    src/Mavlink.cpp
    src/LinkMonitor.cpp
    src/Arbiter.cpp
    src/Failsafe.cpp
    src/Config.cpp
    src/SerialTransport.cpp
    src/UdpTransport.cpp
    src/IcmpProbe.cpp
    src/Router.cpp
)
target_include_directories(uav_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(uav_link src/main.cpp)
target_link_libraries(uav_link PRIVATE uav_core)

# --- тести -------------------------------------------------------------
enable_testing()
foreach(t test_mavlink test_arbiter test_config)
    add_executable(${t} tests/${t}.cpp)
    target_link_libraries(${t} PRIVATE uav_core)
    target_include_directories(${t} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    add_test(NAME ${t} COMMAND ${t})
endforeach()

install(TARGETS uav_link RUNTIME DESTINATION bin)
install(FILES config/uav_link.conf DESTINATION /etc/uav_link)
install(FILES systemd/uav-link.service DESTINATION /etc/systemd/system)
