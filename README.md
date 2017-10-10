# youbot_stack
Collection of packages to run the youbot arm in simulation and in hardware for course COMPGX01


Dependencies:
`sudo apt install ros-kinetic-controller-manager ros-kinetic-joint-state-controller ros-kinetic-effort-controllers ros-kinetic-gazebo-ros-control`

Build:
1. Clone repo into catkin workspace (eg. catkin_ws/src)
2. In the youbot_stack directory, run `git submodule update --init`
3. In the root of the workspace (e.g. ~/catkin_ws) run `catkin_make`
4. Run `source devel/setup.bash`

To run the simulator:
1. run `roslaunch youbot_simulator youbot_sim.launch gui:=true`
The gui true tag brings up the gazebo client if you don't want the gazebo client put false.

To  give permission for non-root execution for the robot. (You have to run these everytime the node has changed.)
1. `cd devel/lib/my_pkg`     # cd to the directory with your node
2. `sudo chown root:root my_node` # change ownship to root
3. `sudo chmod a+rx my_node`      # set as executable by all
4. `sudo chmod u+s my_node `      # set the setuid bit

To run the arm:
1. run `roslaunch youbot_joint_control youbot_arm.launch`
