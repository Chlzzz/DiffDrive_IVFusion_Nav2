cmake_minimum_required(VERSION 3.8)
project(normalcam)

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

add_library(NormalCam SHARED
  include/normalcam/NormalCam.hpp
  src/NormalCam.cpp)

target_compile_definitions(NormalCam
  PRIVATE "COMPOSITION_BUILDING_DLL") 
  
ament_target_dependencies(NormalCam
  "rclcpp"
  "rclcpp_components"
  "cv_bridge"
  "sensor_msgs"
  "camera_info_manager"
  "camera_calibration_parsers"
  "image_transport"
  "OpenCV"
)


rclcpp_components_register_nodes(NormalCam "camera::NormalCam")
set(node_plugins "${node_plugins}camera::NormalCam;$<TARGET_FILE:NormalCam>\n")

add_executable(mycam
  src/main.cpp)
  
target_link_libraries(mycam
  NormalCam)

ament_target_dependencies(mycam
  "rclcpp")

install(TARGETS
  NormalCam
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  mycam
  RUNTIME DESTINATION lib/${PROJECT_NAME})


# # Install launch files.
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
