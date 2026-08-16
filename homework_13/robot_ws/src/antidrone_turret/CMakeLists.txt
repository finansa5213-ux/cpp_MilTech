cmake_minimum_required(VERSION 3.20)
project(antidrone_turret)

# C++20 - базовий стандарт курсу.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ament_auto бере залежності з package.xml і зменшує шаблонний CMake-код.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Власні msg/srv генерують C++ типи для типізованих топіків і сервісу.
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
  "msg/ActuatorStatus.msg"
  "msg/GimbalCommand.msg"
  "msg/ServoCommand.msg"
  "msg/TurretStatus.msg"
  "srv/TriggerActuator.srv"
)

ament_auto_add_executable(target_track_publisher_node src/target_track_publisher_node.cpp)
ament_auto_add_executable(actuator_node src/actuator_node.cpp)
ament_auto_add_executable(turret_controller_node src/turret_controller_node.cpp)
ament_auto_add_executable(gimbal_driver_node src/gimbal_driver_node.cpp)
ament_auto_add_executable(yaw_servo_driver_node src/yaw_servo_driver_node.cpp)

target_include_directories(target_track_publisher_node PRIVATE include)
target_include_directories(actuator_node PRIVATE include)
target_include_directories(turret_controller_node PRIVATE include)
target_include_directories(gimbal_driver_node PRIVATE include)
target_include_directories(yaw_servo_driver_node PRIVATE include)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(target_track_publisher_node "${cpp_typesupport_target}")
target_link_libraries(actuator_node "${cpp_typesupport_target}")
target_link_libraries(turret_controller_node "${cpp_typesupport_target}")
target_link_libraries(gimbal_driver_node "${cpp_typesupport_target}")
target_link_libraries(yaw_servo_driver_node "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(provided_logic_test
    test/provided_logic_test.cpp
  )
  target_include_directories(provided_logic_test PRIVATE include)
  target_compile_definitions(provided_logic_test
    PRIVATE ANTIDRONE_TURRET_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  # Тести чистої логіки контролера турелі (без rclcpp).
  ament_add_gtest(turret_logic_test
    test/turret_logic_test.cpp
  )
  target_include_directories(turret_logic_test PRIVATE include)
endif()

ament_export_dependencies(rosidl_default_runtime)

# launch/ і config/ встановлюються у share, щоб ros2 launch знаходив файли.
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE
  launch
  config
  tracks
)
