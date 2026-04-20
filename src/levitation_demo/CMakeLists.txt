
cmake_minimum_required(VERSION 3.5)
project(levitation_demo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

add_library(levitation_plugin SHARED plugins/levitation_plugin.cpp)
ament_target_dependencies(levitation_plugin rclcpp geometry_msgs)
target_link_libraries(levitation_plugin ${GAZEBO_LIBRARIES})

install(TARGETS levitation_plugin
  LIBRARY DESTINATION lib
)

install(PROGRAMS
  scripts/teleop_force_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
