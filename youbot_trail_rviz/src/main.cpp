#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main( int argc, char** argv )
{
    ros::init(argc, argv, "trail_nodes");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Rate r(30);

    rosbag::Bag bag;
    bag.open("/home/kpach/catkin_ws/src/youbot_stack/youbot_simulator/bags/data_q4c.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("target_position"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));


    //Define points message
    visualization_msgs::Marker points;
    points.header.frame_id = "/base_link";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.01;
    points.scale.y = 0.01;

    // Points are red
    points.color.r = 1.0f;
    points.color.a = 1.0;

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
        r.sleep();
    }
}
