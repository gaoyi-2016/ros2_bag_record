cmake_minimum_required(VERSION 3.8)
project(ros2_bag_record)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)

add_executable(main src/main.cpp)
target_link_libraries(main
  yaml-cpp
)
ament_target_dependencies(main 
  rclcpp 
  rosbag2_cpp 
  sensor_msgs
)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()