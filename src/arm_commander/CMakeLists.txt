cmake_minimum_required(VERSION 3.8)
project(arm_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robp_interfaces REQUIRED)

#this lines are used to create an executable from the test_moveit.cpp file and link it with the necessary dependencies
add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface )

#this lines are used to create an executable from the test_pickup.cpp file and link it with the necessary dependencies
add_executable(test_pickup src/test_pickup.cpp)
ament_target_dependencies(test_pickup rclcpp moveit_ros_planning_interface tf2_ros geometry_msgs)

#this lines are used to create an executable from the ArmCommander.cpp file and link it with the necessary dependencies
add_executable(arm_commander src/ArmCommander.cpp)
ament_target_dependencies(arm_commander rclcpp moveit_ros_planning_interface tf2_ros geometry_msgs robp_interfaces)

install(TARGETS
  test_moveit
  test_pickup
  arm_commander
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
