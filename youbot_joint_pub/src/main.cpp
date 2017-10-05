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

float joint;

using namespace std;
	
int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_node");

	ros::NodeHandle nhJointPub_;
	ros::Publisher  JointPublisher;

	JointPublisher   = nhJointPub_.advertise<std_msgs::Float32MultiArray>("JointCMD", 1);

	std_msgs::Float32MultiArray JointData;	

	while (nhJointPub_.ok())
	{
		JointData.data.clear();
		cout << "enter the targeted joint positions in degree separate by space: ";
		for (int i = 0; i < 5; i++) 
		{
    			try
			{
				cin >> joint;
				JointData.data.push_back(joint);
			}
			catch ( exception& ex )
			{
				cout << "Error: Invalid input." <<  endl;
				return 0;
			}
		}

		cout << endl;

		JointPublisher.publish(JointData);
						
		ros::spinOnce();
		usleep(3);
	}

	return 0;
}
