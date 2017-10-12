cmake_minimum_required(VERSION 2.8.3)
project(youbot_joint_control)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	trajectory_msgs
	sensor_msgs
)

find_package(youbot_driver REQUIRED)

include_directories(
	${catkin_INCLUDE_DIRS}
	${youbot_driver_DIRS}
)

catkin_package(
	CATKIN_DEPENDS
	roscpp
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
	${youbot_driver_LIBRARIES}
)
