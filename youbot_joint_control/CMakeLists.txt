cmake_minimum_required(VERSION 2.8.3)
project(youbot_joint_control)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	trajectory_msgs
	sensor_msgs
	youbot_driver
)

catkin_package(
#	INCLUDE_DIRS
#	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp trajectory_msgs sensor_msgs youbot_driver
#	DEPENDS
)

include_directories(
	${catkin_INCLUDE_DIRS}
)


link_directories(
	${catkin_LIBRARY_DIRS}
)

add_definitions(-DYOUBOT_CONFIGURATIONS_DIR="${PROJECT_SOURCE_DIR}/config")


add_executable(youbot_joint_control
  src/main.cpp
)


target_link_libraries(youbot_joint_control
	${catkin_LIBRARIES}
	${youbot_driver_SOURCE_DIR}/lib/libYouBotDriver.so
)
