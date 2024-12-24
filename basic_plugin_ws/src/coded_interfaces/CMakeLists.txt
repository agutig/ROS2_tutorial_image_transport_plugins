cmake_minimum_required(VERSION 3.8)
project(coded_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # Needed to add to use interfaces
find_package(builtin_interfaces REQUIRED)         # Needed to add to use interfaces


# Add this lines too
rosidl_generate_interfaces(${PROJECT_NAME}
 "msg/Basic.msg"                                  # Adding the created interfaces
 DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime) # This line is also necesary

ament_package()