To  give permission for non-root execution for the robot. (You have to run these everytime the node has changed.)
1. `cd devel/lib/my_pkg`     # cd to the directory with your node
2. `sudo chown root:root my_node` # change ownship to root
3. `sudo chmod a+rx my_node`      # set as executable by all
4. `sudo chmod u+s my_node `      # set the setuid bit

To run the arm:
1. run `roslaunch youbot_joint_control youbot_arm.launch`
