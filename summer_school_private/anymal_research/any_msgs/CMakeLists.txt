# Minimum cmake version
cmake_minimum_required(VERSION 2.8.3)

# Create project
project(any_msgs)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    message_generation
    sensor_msgs
)

# Declare the message and service files to be built
add_message_files(
  FILES
    Event.msg
    State.msg
    UserInteractionOption.msg
    ExtendedJointState.msg
    Float64Stamped.msg
    ImuWithTrigger.msg
    PointContact.msg
    SensorTimeInfo.msg
)
add_service_files(
  FILES
    ProcessFile.srv
    SetFloat32.srv
    SetFloat64.srv
    SetPose.srv
    SetTopic.srv
    SetUInt32.srv
    Toggle.srv
    UserInteraction.srv
    Shutdown.srv
    SetString.srv
    GetString.srv
)
generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

# Generate a package
catkin_package(
  CATKIN_DEPENDS
    std_msgs
    message_runtime
)
