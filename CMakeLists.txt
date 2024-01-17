cmake_minimum_required(VERSION 3.8)
project(pgv100)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(pgv100 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PGVCommand.msg"
  "msg/PGVScan.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

add_executable(pgv-100 src/pgv-100.cpp)
target_link_libraries(pgv-100 ${catkin_LIBRARIES})
ament_target_dependencies(pgv-100 rclcpp std_msgs serial pgv100)

install(TARGETS
        pgv-100
        DESTINATION lib/${PROJECT_NAME}
)   

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
