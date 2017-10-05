cmake_minimum_required(VERSION 2.8.3)
project(youbot_joint_pub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
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

add_executable(youbot_joint_pub
  src/main.cpp
)

target_link_libraries(youbot_joint_pub
	${catkin_LIBRARIES}
)
