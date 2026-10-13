cmake_minimum_required(VERSION 3.8)
project(sensor_watchdog)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(watchdog_node src/watchdog.cpp)
target_compile_features(watchdog_node PUBLIC cxx_std_17)
target_include_directories(watchdog_node PRIVATE include)

ament_target_dependencies(watchdog_node
  rclcpp
  diagnostic_msgs
  lifecycle_msgs
)
target_link_libraries(watchdog_node Threads::Threads)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(watchdog_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS watchdog_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config systemd DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/power_cycle.sh.example DESTINATION lib/${PROJECT_NAME})

# ROS 없이 도는 판정 로직 테스트
if(BUILD_TESTING)
  add_executable(test_health test/test_health.cpp)
  target_compile_features(test_health PUBLIC cxx_std_17)
  target_include_directories(test_health PRIVATE include)
  add_test(NAME test_health COMMAND test_health)
endif()

ament_package()
