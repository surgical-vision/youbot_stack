#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

visualization_msgs::Marker robot_trail;
geometry_msgs::Point gaz_point;

void update_line(const gazebo_msgs::LinkStates::ConstPtr &pos)
{
    int L = pos->pose.size();
    pos->pose.back().position.x;

    gaz_point.x = 0.5*(pos->pose.at(L - 1).position.x + pos->pose.at(L - 2).position.x);
    gaz_point.y = 0.5*(pos->pose.at(L - 1).position.y + pos->pose.at(L - 2).position.y);
    gaz_point.z = 0.5*(pos->pose.at(L - 1).position.z + pos->pose.at(L - 2).position.z);

    robot_trail.points.push_back(gaz_point);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "trail_nodes");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    ros::Subscriber traj_sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, update_line);
    ros::Rate r(30);

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


    // POINTS markers use x and y scale for width/height respectively
    robot_trail.scale.x = 0.01;

    // Points are red
    robot_trail.color.b = 1.0f;
    robot_trail.color.a = 1.0;

    rosbag::Bag bag;
    bag.open("/home/kpach/catkin_ws/src/youbot_stack/youbot_simulator/bags/data_q4c.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
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

    bag.close();

    while (ros::ok())
    {
        marker_pub.publish(points);
        marker_pub.publish(robot_trail);

        ros::spinOnce();

        r.sleep();
    }
}
