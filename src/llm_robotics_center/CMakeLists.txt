cmake_minimum_required(VERSION 3.8)
project(llm_robotics_center)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(franka_msgs REQUIRED)

# main control node
add_library(llm_robotics_center SHARED src/llm_robotics_center.cpp)
target_include_directories(llm_robotics_center PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(llm_robotics_center
  "communication_interfaces"
  "franka_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
)
rclcpp_components_register_node(llm_robotics_center PLUGIN "llm_robotics_center::LLMRoboticsCenter" EXECUTABLE llm_robotics_control)


# ik controller test
add_library(action_client SHARED src/action_client_test.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(action_client
  "communication_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "move_target_action::MoveTargetActionClient" EXECUTABLE robot_action_client)


# cartesian controller test
add_library(cartesian_client SHARED src/cartesian_client_test.cpp)
target_include_directories(cartesian_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cartesian_client
  "communication_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(cartesian_client PLUGIN "cartesian_move_action::CartesianActionClient" EXECUTABLE cartesian_client_test)


# franka gripper test
add_library(franka_gripper_test SHARED
  src/gripper_client_test.cpp)
target_include_directories(franka_gripper_test PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(franka_gripper_test
  "franka_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
)
rclcpp_components_register_node(franka_gripper_test PLUGIN "franka_gripper_client::GripperActionClient" EXECUTABLE franka_gripper)

install(TARGETS
  cartesian_client
  franka_gripper_test
  llm_robotics_center
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
