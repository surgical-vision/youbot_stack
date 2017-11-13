#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <inverse_kinematics/YoubotKDL.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

visualization_msgs::Marker robot_trail;
geometry_msgs::Point gaz_point;

void update_line(const gazebo_msgs::LinkStates::ConstPtr &pos)
{
    int L = pos->pose.size();

    gaz_point.x = 0.5*(pos->pose.at(L - 1).position.x + pos->pose.at(L - 2).position.x);
    gaz_point.y = 0.5*(pos->pose.at(L - 1).position.y + pos->pose.at(L - 2).position.y);
    gaz_point.z = 0.5*(pos->pose.at(L - 1).position.z + pos->pose.at(L - 2).position.z);

    robot_trail.points.push_back(gaz_point);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "trail_node");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    ros::Subscriber traj_sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, update_line);
    ros::Rate r(30);

    int checkpoint_data = atoi(argv[1]);

    YoubotKDL youbot;
    youbot.init();
    KDL::Frame current_pose;

    //Define points message
    visualization_msgs::Marker points;
    points.header.frame_id = robot_trail.header.frame_id = "/base_link";
    points.header.stamp = robot_trail.header.stamp = ros::Time::now();
    points.ns = robot_trail.ns = "points_and_lines";
    points.action = robot_trail.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = robot_trail.pose.orientation.w =1.0;

    points.id = 0;
    robot_trail.id = 1;

    points.type = visualization_msgs::Marker::POINTS;
    robot_trail.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.01;
    points.scale.y = 0.01;

    // Points are red
    points.color.g = 1.0f;
    points.color.a = 1.0;


    robot_trail.scale.x = 0.005;

    // Robot trails are blue
    robot_trail.color.b = 1.0f;
    robot_trail.color.a = 1.0;

    rosbag::Bag bag;

    switch (checkpoint_data)
    {
        case 1: bag.open(MY_BAG_PATH1, rosbag::bagmode::Read); break;
        case 2: bag.open(MY_BAG_PATH2, rosbag::bagmode::Read); break;
        case 3: bag.open(MY_BAG_PATH3, rosbag::bagmode::Read); break;
        case 4: bag.open(MY_BAG_PATH4, rosbag::bagmode::Read); break;
        case 5: bag.open(MY_BAG_PATH5, rosbag::bagmode::Read); break;
        default: ROS_ERROR("Invalid input."); return 0; break;
    }

    std::vector<std::string> topics;

    if (checkpoint_data == 1)
    {
        topics.push_back(std::string("joint_data"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::JointState::ConstPtr s = m.instantiate<sensor_msgs::JointState>();
            if (s != NULL)
            {

                KDL::JntArray joint;
                joint.resize(5);
                for (int i = 0; i < 5; i++)
                    joint.data(i) = s->position.at(i);

                current_pose = youbot.forward_kinematics(joint, youbot.current_pose);

                geometry_msgs::Point p;
                p.x = current_pose.p.x();
                p.y = current_pose.p.y();
                p.z = current_pose.p.z();
                points.points.push_back(p);
            }

        }
    }
    else if (checkpoint_data == 2)
    {
        topics.push_back(std::string("target_tf"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach(rosbag::MessageInstance const m, view)
        {
            geometry_msgs::TransformStamped::ConstPtr s = m.instantiate<geometry_msgs::TransformStamped>();
            if (s != NULL)
            {
                geometry_msgs::Point p;
                p.x = s->transform.translation.x;
                p.y = s->transform.translation.y;
                p.z = s->transform.translation.z;
                points.points.push_back(p);
            }

        }
    }
    else
    {
        topics.push_back(std::string("target_position"));
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach(rosbag::MessageInstance const m, view)
        {
            geometry_msgs::Point::ConstPtr s = m.instantiate<geometry_msgs::Point>();
            if (s != NULL)
            {
                geometry_msgs::Point p;
                p.x = s->x;
                p.y = s->y;
                p.z = s->z;
                points.points.push_back(p);
            }

        }
    }

    bag.close();

    while (ros::ok())
    {
        marker_pub.publish(points);
        marker_pub.publish(robot_trail);

        ros::spinOnce();

        r.sleep();
    }
}
