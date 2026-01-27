cmake_minimum_required(VERSION 3.5)
project(inha_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Speak.action"
  "action/Listen.action"
  "action/CaptureFaceCrop.action"
  "action/WaitPerson.action"
  "action/Vlm.action"
  "action/SetRobotPose.action"
  "action/FollowHuman.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
