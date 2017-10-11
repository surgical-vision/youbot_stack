#!/usr/bin/env python

import rospy
import math
from trajectory_msgs.msg import JointTrajectoryPoint


def timer_callback(event):

    global current_time, joint

    for i in range(0, 5):
        joint[i] = 0.5*(theta_max[i] - theta_min[i])*math.cos(2*math.pi*current_time/period_joint[i]) \
                   + 0.5*(theta_max[i] + theta_min[i])

    if current_time < 100*math.pi:
        current_time = current_time + 0.01
    else:
        current_time = 0.0


joint = [169.0, -65.0, 146.0, -102.5, 165.00]
current_time = 0.0
theta_max = [169.0, -45.0, 146.0, -40.5, 165.0]
theta_min = [60.0, 0.0, 86.0, -102.5, 0.0]
period_joint = [8.7, 5.8, 5, 6, 9.5]

rospy.init_node('cmd_joint_node')

joint_publisher = rospy.Publisher('/arm/cmd_joint_traj', JointTrajectoryPoint, queue_size=1)
rospy.Timer(rospy.Duration(0.01), timer_callback)
rate = rospy.Rate(1000)

joint_data = JointTrajectoryPoint()

while not rospy.is_shutdown():

    joint_data.positions = joint
    joint_publisher.publish(joint_data)
    rate.sleep()
