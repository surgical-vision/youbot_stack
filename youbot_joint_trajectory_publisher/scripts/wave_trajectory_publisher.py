import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState


def msg_generator(joint_name, joints_pos):
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()

    msg.joint_names = joint_name

    point = JointTrajectoryPoint()
    point.positions = joints_pos
    point.time_from_start.nsecs = 537230041

    msg.points.append(point)
    return msg


if __name__ == '__main__':
    rospy.init_node('youbot_trajectory_publisher')

    joint_names = rospy.get_param('/EffortJointInterface_trajectory_controller/joints')

    traj_publisher = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory,
                                     queue_size=3)

