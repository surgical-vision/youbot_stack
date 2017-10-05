cmake_minimum_required(VERSION 2.8.3)
project(youbot_joint_control)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	youbot_driver
)

include_directories(
	INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
)

catkin_package(
	CATKIN_DEPENDS
	roscpp
	std_msgs
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
	${PROJECT_SOURCE_DIR}/lib/libYouBotDriver.so
)
