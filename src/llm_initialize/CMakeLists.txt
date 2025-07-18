cmake_minimum_required(VERSION 3.8)
project(llm_initialize)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(communication_interfaces REQUIRED)

add_executable(llm_output src/llm_initialize.cpp)
ament_target_dependencies(llm_output rclcpp communication_interfaces)

add_executable(llm_process src/llm_subscriber_test.cpp)
ament_target_dependencies(llm_process
  rclcpp
  communication_interfaces
)

add_executable(output src/llm_output_test.cpp)
ament_target_dependencies(output
  rclcpp
  communication_interfaces
)


install(TARGETS
  output
  llm_process
  llm_output
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
