cmake_minimum_required(VERSION 3.8)
project(open_manipulator_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake               REQUIRED)
find_package(rclcpp                    REQUIRED)
find_package(rclcpp_action             REQUIRED)
find_package(rclcpp_components         REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs               REQUIRED)
find_package(std_msgs                  REQUIRED)
find_package(sensor_msgs               REQUIRED)
# MoveIt
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core                   REQUIRED)
# Still needed for the bundled open_manipulator_msgs package
find_package(open_manipulator_msgs     REQUIRED)

# ── Generate this package's own action interfaces ─────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPosition.action"
  "action/GripperControl.action"
  DEPENDENCIES action_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ── arm_controller shared library ─────────────────────────────────────────────
add_library(arm_controller_lib SHARED
  src/arm_controller.cpp
)
target_include_directories(arm_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(arm_controller_lib
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  moveit_ros_planning_interface
  moveit_core
)
target_link_libraries(arm_controller_lib
  "${cpp_typesupport_target}"
)

# ── arm_controller_node executable ───────────────────────────────────────────
add_executable(arm_controller_node
  src/arm_controller_node.cpp
)
ament_target_dependencies(arm_controller_node rclcpp)
target_link_libraries(arm_controller_node
  arm_controller_lib
)

# ── example_client executable ─────────────────────────────────────────────────
add_executable(example_client
  src/example_client.cpp
)
target_include_directories(example_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(example_client
  rclcpp
  rclcpp_action
)
target_link_libraries(example_client
  "${cpp_typesupport_target}"
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  arm_controller_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  arm_controller_node
  example_client
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
