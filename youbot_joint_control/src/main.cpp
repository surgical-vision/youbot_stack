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

#include "youbot/YouBotManipulator.hpp"

#define PI  3.14159265358979323846


using namespace std;

float theta_init[5] = {169.0*PI/180.0, -65.0*PI/180.0, 146.0*PI/180.0, -102.5*PI/180.0, 165.0*PI/180.0};
float max_lim_youbot[5] = {5.84013, 2.6178, -0.015709, 3.4291, 5.6414};
float min_lim_youbot[5] = {0.01007, 0.01007, -5.0264, 0.022124, 0.11062};		


class YouBotNode
{
	private: 
		ros::NodeHandle nhYoubot_;
		ros::Publisher  JointPublisher;
		ros::Subscriber JointSubscriber;

		youbot::YouBotManipulator * youBotArm;
	
		// variables used to get and set jointAngles or set velocity of the arm
		youbot::JointSensedAngle    sensedAngle;
		youbot::JointSensedVelocity sensedVelocity;

		// to open or close the gripper
		youbot::GripperBarSpacingSetPoint barSpacing;

	public: 
		YouBotNode() : nhYoubot_() {}

		int main(int argc, char **argv)
		{

			//check if arm hardware exists
			try
			{
				youBotArm  = new youbot::YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
				youBotArm->doJointCommutation();
				cout << "Finish Commutation" << endl;
   				youBotArm->calibrateManipulator();
				cout << "Finish Calibration" << endl;
				barSpacing.barSpacing = 0.0 * meter;
				youBotArm->getArmGripper().setData(barSpacing);

			}
			catch ( exception& ex )
			{
				cout << "ARM Exception: " << ex.what() << endl;
			}

			JointPublisher   = nhYoubot_.advertise<std_msgs::Float32MultiArray>("JointData", 1);
			JointSubscriber  = nhYoubot_.subscribe<std_msgs::Float32MultiArray>("JointCMD", 1, boost::bind(&YouBotNode::MoveArm, this, _1, youBotArm));
			std_msgs::Float32MultiArray JointData;	

			while (nhYoubot_.ok())
			{
				JointData.data.clear();
				for (int i = 0; i < 5; i++)
				{
					youBotArm->getArmJoint(i + 1).getData(sensedAngle);
					youBotArm->getArmJoint(i + 1).getData(sensedVelocity);
					if ((i == 0) || (i == 4))
						JointData.data.push_back(theta_init[i] - sensedAngle.angle.value());
					else
						JointData.data.push_back(theta_init[i] + sensedAngle.angle.value());
					JointData.data.push_back(sensedVelocity.angularVelocity.value());
				}

				JointPublisher.publish(JointData);
						
				ros::spinOnce();
				usleep(3);
			}

			return 0;
		}

		void MoveArm(const std_msgs::Float32MultiArray::ConstPtr &joint_cmd, youbot::YouBotManipulator *youBotArm)
		{
			int cc = 0;
			float incoming_joint;
			youbot::JointAngleSetpoint setAngle;

			for (std::vector<float>::const_iterator it = joint_cmd->data.begin(); it != joint_cmd->data.end(); ++it)
			{
				incoming_joint = *it;
				//Apply joint offset
				if ((cc == 0) || (cc == 4))
					incoming_joint = theta_init[cc] - incoming_joint*PI/180.0;
				else
					incoming_joint = incoming_joint*PI/180.0 - theta_init[cc];
			
				//Make sure the input does not exceed joint limit
				if (incoming_joint < min_lim_youbot[cc])
					incoming_joint = min_lim_youbot[cc];
				else if (incoming_joint > max_lim_youbot[cc])
					incoming_joint = max_lim_youbot[cc];

				setAngle.angle = incoming_joint * radian;
				youBotArm->getArmJoint(cc + 1).setData(setAngle);
				cc = cc + 1;
			}
		}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "youbot_node");
	YouBotNode Y;
	return Y.main(argc, argv);
}
