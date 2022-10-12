#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <iostream>
#include <vector>
#include <boost/bind.hpp>

using std::cout;
using std::endl;

// move_base需要map -> /odom的tf关系
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg, int& id){
    static tf::TransformBroadcaster tf_map_odom_br;
    static tf::TransformBroadcaster tf_odom_foot_br;
    static ros::Publisher realOdom_pub;
    static ros::Publisher realFoot_pub;

    tf::Transform tf_map_odom;
    tf_map_odom.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf_map_odom.setRotation(q);

    ros::Time t = ros::Time::now();
    std::string odom_name = "robot" + std::to_string(id + 1) + "_tf/odom";
    tf_map_odom_br.sendTransform(tf::StampedTransform(tf_map_odom, ros::Time::now() + ros::Duration(0.1), "map", odom_name));
    //tf_odom_foot_br.sendTransform(tf::StampedTransform(tf_odom_foot, t, "robot1_tf/odom", "robot1_tf/base_footprint"));
}

int main(int argc, char** argv){
    ros::init(argc, argv,"PublishGroundTruth");

    ros::NodeHandle n;
    int robot_num = 0;
    if(!n.getParam("/robot_number", robot_num)){
        ROS_ERROR("Please set your robot number.");
        return -1;
    }

    std::vector<ros::Subscriber> odom_subs(robot_num);
    for(int i = 0; i < robot_num; ++i){
        std::string groundtruth_name = "robot" + std::to_string(i + 1) + "/ground_truth/state";
        odom_subs[i] = n.subscribe<nav_msgs::Odometry>(groundtruth_name, 5, boost::bind(odom_callback, _1, i));
    }

    int hertz = 60;
    ros::Rate r(hertz);
    while(ros::ok()){
        ros::spinOnce();

        r.sleep();
    }

    return 0;
}