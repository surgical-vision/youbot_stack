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

#define q4a 1
//#define q4b 1
//#define q4cd_extra 1

double DH_param[4][5] = {{0.033, 0.155, 0.135, 0.0, 0.0}, {M_PI_2, 0.0, 0.0, M_PI_2, 0.0}, {0.147, 0.0, 0.0, 0.0, 0.183},
                         {170*M_PI/180, 65*M_PI/180+M_PI_2, -146*M_PI/180, M_PI_2+102.5*M_PI/180, M_PI+167.5*M_PI/180}};

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

Eigen::Matrix4d DH_mat(double a, double alpha, double d, double theta)
{
    Eigen::Matrix4d A;
    A(0, 0) = cos(theta); A(0, 1) = -sin(theta)*cos(alpha); A(0, 2) =  sin(theta)*sin(alpha); A(0, 3) = a*cos(theta);
    A(1, 0) = sin(theta); A(1, 1) =  cos(theta)*cos(alpha); A(1, 2) = -cos(theta)*sin(alpha); A(1, 3) = a*sin(theta);
    A(2, 0) = 0.0;        A(2, 1) =  sin(alpha);            A(2, 2) =  cos(alpha);            A(2, 3) = d;
    A(3, 0) = 0.0;        A(3, 1) =  0.0;                   A(3, 2) =  0.0;                   A(3, 3) = 1.0;
    return A;
}

Eigen::Matrix4d fkine(double theta[])
{
    Eigen::Matrix4d T;
    T.setIdentity();
    for (int i = 0; i < 5; i++)
        T = T*DH_mat(DH_param[0][i], DH_param[1][i], DH_param[2][i], DH_param[3][i] - theta[i]);
    return T;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "trail_nodes");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    ros::Subscriber traj_sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, update_line);
    ros::Rate r(30);

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

    // Robot trails are green
    robot_trail.color.g = 1.0f;
    robot_trail.color.a = 1.0;

    rosbag::Bag bag;
    bag.open(MY_BAG_PATH, rosbag::bagmode::Read);

    std::vector<std::string> topics;

#ifdef q4a
    topics.push_back(std::string("joint_data"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::JointState::ConstPtr s = m.instantiate<sensor_msgs::JointState>();
        if (s != NULL)
        {

            double input_j[5];
            for (int i = 0; i < 5; i++)
               input_j[i] = s->position.at(i);

            Matrix4d T = fkine(input_j);

            geometry_msgs::Point p;
            p.x = T(0, 3);
            p.y = T(1, 3);
            p.z = T(2, 3);
            points.points.push_back(p);
        }

    }
#elif q4b
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
#elif q4cd_extra
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
#endif

    bag.close();

    while (ros::ok())
    {
        marker_pub.publish(points);
        marker_pub.publish(robot_trail);

        ros::spinOnce();

        r.sleep();
    }
}
