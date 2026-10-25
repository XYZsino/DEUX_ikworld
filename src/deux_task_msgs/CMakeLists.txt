cmake_minimum_required(VERSION 3.8)
project(deux_task_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectDetection.msg" "msg/ObjectDetectionArray.msg" "msg/Observation.msg"
  "msg/BoundingBox.msg" "msg/DetectionFrame.msg"
  "msg/StateObservation.msg" "msg/PerceptionRequest.msg"
  "msg/DetectionArray.msg" "msg/BarcodeTarget.msg"
  "msg/BarcodeRead.msg"
  "msg/ScanSession.msg"
  "msg/MotionPlan.msg"
  "msg/TaskStatus.msg"
  "action/PlanMotion.action"
  "action/ExecuteMotion.action"
  "action/BridgeMotion.action"
  "action/RunTask.action"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs trajectory_msgs builtin_interfaces action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
