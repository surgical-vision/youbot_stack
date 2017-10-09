
# youbot_stack
Collection of packages to run the youbot arm in simulation and in hardware for course COMPGX01
Dependencies:
`sudo apt install ros-kinetic-controller-manager ros-kinetic-joint-state-controller`
updating ros directory:
`export ROS_PACKAGE_PATH=~/catkin_ws/src:$ROS_PACKAGE_PATH`
Build:
1. clone into catkin workspace (eg. catkin_ws/src)
2. run `git submodule update --init`
3. run `catkin_make`
