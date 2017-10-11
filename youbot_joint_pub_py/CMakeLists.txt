cmake_minimum_required(VERSION 2.8.3)
project(youbot_joint_pub_py)

find_package(catkin REQUIRED
	rospy
	trajectory_msgs
)

catkin_package(
	CATKIN_DEPENDS rospy
)

catkin_install_python(PROGRAMS
	script/main.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
