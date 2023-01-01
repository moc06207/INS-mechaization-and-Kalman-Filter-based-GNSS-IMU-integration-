cmake_minimum_required(VERSION 3.0.2)
project(INS)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)





# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(

  LIBRARIES Localization
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# catkin_install_python(PROGRAMS
#   src/UKF_stand_alone_gps_Imu_integration/ukf_main_0801_1.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

