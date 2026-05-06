cmake_minimum_required(VERSION 3.8)
project(scara_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


install(
  DIRECTORY
    urdf
    launch
    config
    rviz
    meshes
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(move src/move.cpp)
ament_target_dependencies(move
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(pick_and_place src/pick_and_place.cpp)
ament_target_dependencies(pick_and_place
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  visualization_msgs
)

add_executable(cam_pick_and_place src/cam_pick_and_place.cpp)
ament_target_dependencies(cam_pick_and_place
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  move
  pick_and_place
  cam_pick_and_place
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/fkpos.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/ikpos.py
  DESTINATION lib/${PROJECT_NAME}
)

# install(PROGRAMS
#   ${PROJECT_NAME}/ik_arm.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   ${PROJECT_NAME}/relay_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   ${PROJECT_NAME}/trajectory_planner.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   ${PROJECT_NAME}/dh_panda.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   ${PROJECT_NAME}/erobot.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   ${PROJECT_NAME}/move.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   ${PROJECT_NAME}/move_v2.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   ${PROJECT_NAME}/move_v3.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Optional: install individual files if you have loose ones (e.g. README)
# install(FILES README.md DESTINATION share/${PROJECT_NAME})

# ────────────────────────────────────────────────
# Testing / linting (good practice – optional but recommended)
# ────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# This must be the last line!
ament_package()
