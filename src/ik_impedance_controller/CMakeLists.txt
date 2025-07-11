cmake_minimum_required(VERSION 3.8)
project(ik_impedance_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(communication_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)


# 构建共享库
add_library(
  ${PROJECT_NAME}
  SHARED
  src/ik_impedance_controller.cpp
  src/motion_generator.cpp
)

# 包含目录配置
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 导出插件描述文件
pluginlib_export_plugin_description_file(
  controller_interface 
  controllers_plugin.xml
)

# 链接依赖库
ament_target_dependencies(
  ${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  franka_semantic_components
  moveit_core
  moveit_msgs
  communication_interfaces
  rclcpp_action
  rclcpp_components
)

# 安装规则
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

target_include_directories(ik_impedance_controller PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(ik_impedance_controller
  PRIVATE "IK_IMPEDANCE_CONTROLLER_BUILDING_DLL"
)

rclcpp_components_register_node(
  ik_impedance_controller
  PLUGIN "ik_impedance_controller::IkImpedanceController" 
  EXECUTABLE ik_controller
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  moveit_core
)

ament_package()