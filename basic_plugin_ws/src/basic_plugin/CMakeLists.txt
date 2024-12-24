cmake_minimum_required(VERSION 3.8)
project(basic_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies, the same like always
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)              #Required for plugins
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(coded_interfaces REQUIRED)       #Your personalized interface


# Add the created code (Remember manifest.cpp!)
add_library(${PROJECT_NAME} SHARED
  src/basic_publisher.cpp
  src/basic_subscriber.cpp
  src/manifest.cpp
)

# Add the dependencies (yes,again)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  image_transport
  sensor_msgs
  cv_bridge
  OpenCV
  coded_interfaces
)

# Add this lines too, they are key for plugins
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  plugins.xml
  DESTINATION share/${PROJECT_NAME}
)


include_directories(
  include
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
################################################


#Export the plugin --> pluginlib_export_plugin_description_file(original_node plugins.xml)
pluginlib_export_plugin_description_file(image_transport plugins.xml)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
