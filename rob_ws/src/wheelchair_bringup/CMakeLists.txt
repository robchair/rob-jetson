cmake_minimum_required(VERSION 3.5)
project(wheelchair_bringup)

find_package(ament_cmake REQUIRED)
find_package(robot_localization REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()