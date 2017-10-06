#include <unistd.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <stdint.h>

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#define PI  3.14159265358979323846

float joint[5] = {169.0, -65.0, 146.0, -102.5, 165.0};
float current_t = 0.0;
float theta_max[5] = {169.0, -45, 146, -40.5, 165.0};
float theta_min[5] = {100.0, 0.0, 86, -102.5, 100.0};
float period_joint[5] = {6.7, 5.5, 5, 6, 5.8};

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
	ros::init(argc, argv, "joint_node");

	ros::NodeHandle nhJointPub_;
	ros::Publisher  JointPublisher;

	JointPublisher   = nhJointPub_.advertise<std_msgs::Float32MultiArray>("JointCMD", 1);
	ros::Timer timer1 = nhJointPub_.createTimer(ros::Duration(0.01), timer_callBack);

	std_msgs::Float32MultiArray JointData;	

	while (ros::ok())
	{
		JointData.data.clear();
		for (int i = 0; i < 5; i++) 
			JointData.data.push_back(joint[i]);


		JointPublisher.publish(JointData);
						
		ros::spinOnce();
		usleep(3);
	}

	return 0;
}


