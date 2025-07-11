cmake_minimum_required(VERSION 3.8)
project(franka_gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(Franka 0.13.2 REQUIRED)

add_library(gripper_server SHARED
        src/gripper_action_server.cpp)
target_link_libraries(gripper_server Franka::Franka)
target_include_directories(gripper_server PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(gripper_server
        franka_msgs
        rclcpp
        rclcpp_action
        rclcpp_components
        Franka
        sensor_msgs
        std_srvs
        control_msgs)
rclcpp_components_register_node(gripper_server PLUGIN "franka_gripper::GripperActionServer" EXECUTABLE franka_gripper_node)
install(TARGETS
        gripper_server
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY
        launch config
        DESTINATION share/${PROJECT_NAME}/)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
        scripts/fake_gripper_state_publisher.py
        DESTINATION lib/${PROJECT_NAME})

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
