cmake_minimum_required(VERSION 3.8)
project(ipcamera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(include)

set(node_plugins "")

add_library(IpCamRos2 SHARED
  src/IpCamera.cpp)
target_compile_definitions(IpCamRos2
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(IpCamRos2
  "rclcpp"
  "rclcpp_components"
  "cv_bridge"
  "sensor_msgs"
  "camera_info_manager"
  "camera_calibration_parsers"
  "image_transport"
  "OpenCV"
)

rclcpp_components_register_nodes(IpCamRos2 "camera::IpCamera")
set(node_plugins "${node_plugins}camera::IpCamera;$<TARGET_FILE:IpCamRos2>\n")


add_executable(${PROJECT_NAME}
  src/CameraNode.cpp)

target_link_libraries(${PROJECT_NAME}
  IpCamRos2)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp")

install(TARGETS
  IpCamRos2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
