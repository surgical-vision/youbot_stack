#include <unistd.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <stdint.h>

#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>

#define PI  3.14159265358979323846

double joint[5] = {169.0, -65.0, 146.0, -102.5, 165.0};
double current_t = 0.0;
double theta_max[5] = {169.0, -45.0, 146.0, -40.5, 165.0};
double theta_min[5] = {60.0, 0.0, 86.0, -102.5, 0.0};
double period_joint[5] = {8.7, 5.8, 5, 6, 9.5};

using namespace std;

void timer_callBack(const ros::TimerEvent&)
{
	for (int i = 0; i < 5; i++)
		if (i != 2)
			joint[i] = 0.5*(theta_max[i] - theta_min[i])*cos(2*PI*current_t/period_joint[i]) + 0.5*(theta_max[i] + theta_min[i]);
	if (current_t < 100*PI)
		current_t = current_t + 0.01;
	else
		current_t = 0.0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cmd_joint_node");

	ros::NodeHandle nhJointPub_;
	ros::Publisher  JointPublisher;

	JointPublisher   = nhJointPub_.advertise<trajectory_msgs::JointTrajectoryPoint>("/arm/cmd_joint_traj", 1);
	ros::Timer timer1 = nhJointPub_.createTimer(ros::Duration(0.01), timer_callBack);

	trajectory_msgs::JointTrajectoryPoint JointData;	

	while (ros::ok())
	{
		JointData.positions.clear();
		for (int i = 0; i < 5; i++) 
			JointData.positions.push_back(joint[i]);


		JointPublisher.publish(JointData);
						
		ros::spinOnce();
		usleep(3);
	}

	return 0;
}


