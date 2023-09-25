cmake_minimum_required(VERSION 3.8)
project(wbr914_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the project path from the enviorment variable
set(WBR914_PROJECT_PATH $ENV{WBR914_PROJECT_PATH})
message(STATUS "WBR914_PROJECT_PATH: ${WBR914_PROJECT_PATH}")

# find dependencies. Add more as needed.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED) ## For interfaces
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add custom interfaces - services(srv) and messages(msg) here
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VelocityGet.srv"
  "srv/PositionGet.srv"
  DEPENDENCIES geometry_msgs
)

include_directories(
include/wbr914_package
${WBR914_PROJECT_PATH}/wbr914_base_driver
)

# Nodes executables
add_executable(wbr914_node
src/wbr914_node.cpp
include/wbr914_package/wbr914_node.h
${WBR914_PROJECT_PATH}/wbr914_base_driver/wbr914_minimal.h
${WBR914_PROJECT_PATH}/wbr914_base_driver/wbr914_minimal.cpp)

# Add needed ROS packages to the executables
ament_target_dependencies(wbr914_node rclcpp geometry_msgs)
rosidl_target_interfaces(wbr914_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
 wbr914_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
