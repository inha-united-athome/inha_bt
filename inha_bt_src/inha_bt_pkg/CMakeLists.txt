cmake_minimum_required(VERSION 3.8)
project(inha_bt_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)  
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(inha_interfaces REQUIRED) 
include_directories(${inha_interfaces_INCLUDE_DIRS})
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

set(THIS_PACKAGE_DEPS
    behaviortree_cpp
    behaviortree_ros2
    std_msgs
    std_srvs
    rclcpp_action
    yaml-cpp    
)

add_executable(bt_node src/bt_main.cpp
                       src/navigation/WaitGoalReached.cpp
                       src/navigation/Exploration.cpp
                       src/navigation/GoToPose.cpp
                       src/hri/Speak.cpp
                       src/hri/Listen.cpp
                       src/hri/VlmQuery.cpp
                       src/hri/ExtractWord.cpp
                       src/hri/ContextJsonMaker.cpp
                       src/perception/Face.cpp
                       src/perception/Waving.cpp
                       )

target_link_libraries(bt_node
  yaml-cpp
)
                       
ament_target_dependencies(bt_node
  rclcpp
  std_msgs
  behaviortree_cpp
  behaviortree_ros2
  rclcpp_action
  nav2_msgs
  geometry_msgs
  yaml-cpp
  sensor_msgs
  cv_bridge
  inha_interfaces
  )



install(TARGETS bt_node
  DESTINATION lib/${PROJECT_NAME}
)


target_include_directories(bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY bt_xmls 
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY configs 
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
