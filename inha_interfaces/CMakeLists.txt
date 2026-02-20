cmake_minimum_required(VERSION 3.5)
project(inha_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Speak.action"
  "action/Listen.action"
  "action/CaptureFaceCrop.action"
  "action/WaitPerson.action"
  "action/Vlm.action"
  "action/SetRobotPose.action"
  "action/FollowHuman.action"
  "action/Waving.action"
  "action/WavingApproach.action"
  "action/Detection.action"
  "action/VisualAlign.action"
  "srv/SetEnable.srv"
  "srv/ExecuteSuccess.srv"
  "srv/MoveitEnable.srv"
  "srv/GraspgenEnable.srv"
  "srv/StartReplay.srv"
  "srv/StopReplay.srv"


  DEPENDENCIES builtin_interfaces geometry_msgs
)



ament_export_dependencies(rosidl_default_runtime)
ament_package()
