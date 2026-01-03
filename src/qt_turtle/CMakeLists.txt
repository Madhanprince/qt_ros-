cmake_minimum_required(VERSION 3.5)
project(qt_turtle)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_executable(qt_turtle_node
  src/main.cpp
  resources/resources.qrc
)

ament_target_dependencies(qt_turtle_node rclcpp)
target_link_libraries(qt_turtle_node Qt5::Widgets)

install(TARGETS
  qt_turtle_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
