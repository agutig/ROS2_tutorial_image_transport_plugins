cmake_minimum_required(VERSION 3.8)
project(cam_video_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_publisher src/camera_publisher.cpp)
add_executable(generic_subscriber src/generic_subscriber.cpp)

ament_target_dependencies(camera_publisher rclcpp sensor_msgs image_transport cv_bridge OpenCV)
ament_target_dependencies(generic_subscriber rclcpp)

install(TARGETS
  camera_publisher
  generic_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(camera_publisher curl)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
