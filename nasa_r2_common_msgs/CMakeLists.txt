cmake_minimum_required(VERSION 2.8.3)
project(nasa_r2_common_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation nasa_cmake std_msgs visualization_msgs geometry_msgs trajectory_msgs rospy)

catkin_destinations()

add_message_files(
  DIRECTORY
    msg
  FILES 
    BmsCharge.msg
    BmsConfig.msg
    BmsLimit.msg
    BmsLimitCounter.msg
    BmsManual.msg
    BmsMax.msg
    BmsSlave.msg
    BmsStatus.msg
    CommandStatus.msg
    ControllerJointSettings.msg
    ControllerPoseSettings.msg
    ForceControlAxis.msg
    ForceControlAxisArray.msg
    Gains.msg
    GetParameterRequest.msg
    GetParameterResponse.msg
    GripperEnvironment.msg
    GripperPositionCommand.msg
    GripperPositionState.msg
    GuardDog.msg
    Inertia.msg
    IoChannels.msg
    IoStats.msg
    JointCapability.msg
    JointCommand.msg
    JointControl.msg
    JointControlCalibrationMode.msg
    JointControlClearFaultMode.msg
    JointControlCoeffState.msg
    JointControlCommandMode.msg
    JointControlData.msg
    JointControlDataArray.msg
    JointControlMode.msg
    JointStatus.msg
    JointStatusArray.msg
    JointTrajectoryReplan.msg
    LabeledControllerJointSettings.msg
    LabeledControllerPoseSettings.msg
    LabeledGains.msg
    LabeledGripperPositionCommand.msg
    LabeledJointControlDataArray.msg
    LabeledJointState.msg
    LabeledJointTrajectory.msg
    LabeledPoseTrajectory.msg
    LabeledTrajectoryMonitorFactors.msg
    MStopState.msg
    MagnitudeArray.msg
    MappedType.msg
    MemoryRegion16.msg
    Modes.msg
    NodeRegisterData.msg
    NodeRegisterDataArray.msg
    Parameter.msg
    PoseState.msg
    PoseTrajectory.msg
    PoseTrajectoryPoint.msg
    PoseTrajectoryReplan.msg
    PoseTwistStamped.msg
    PowerCommand.msg
    PowerState.msg
    PriorityArray.msg
    ReplanType.msg
    SetParameterRequest.msg
    SetParameterResponse.msg
    StringArray.msg
    SystemCore.msg
    SystemStatus.msg
    ToolFrame.msg
    TorsoStatus.msg
    TrajectoryMonitorFactors.msg
    ValueType.msg
    VariantMap.msg
    VariantMapArray.msg
    WrenchMonitorLimit.msg
    WrenchState.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES 
    BmsSetLogRate.srv
    BmsSetRosRate.srv
    ClearSoftwareEStop.srv
    GetDouble.srv
    GetLiveParameter.srv
    HasDouble.srv
    Heartbeat.srv
    MStop.srv
    ParseTableScene.srv
    Power.srv
    ResetTableScene.srv
    Power.srv
    ResetTableScene.srv
    SendString.srv
    Servo.srv
    SetControlValue.srv
    SetDouble.srv
    SetIp.srv
    SetJointMode.srv
    SetLiveParameter.srv
    SetTipName.srv
    SetWrenchLimit.srv
    TakeSnapshot.srv
    Trigger.srv
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs visualization_msgs geometry_msgs trajectory_msgs)

catkin_package(
  INCLUDE_DIRS include ${PROTOGEN_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime std_msgs visualization_msgs geometry_msgs trajectory_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/RosMsgPrinter.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
