#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <iostream>

using std::cout;
using std::endl;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    static tf::TransformBroadcaster tf_map_odom_br;
    static tf::TransformBroadcaster tf_odom_foot_br;
    static ros::Publisher realOdom_pub;
    static ros::Publisher realFoot_pub;

    tf::Transform tf_map_odom;
    tf_map_odom.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf_map_odom.setRotation(q);

    // tf::Transform tf_odom_foot;
    // tf_odom_foot.setOrigin(tf::Vector3(0, 0, 0));
    // tf::Quaternion q2;
    // q2.setRPY(0, 0, 0);
    // tf_odom_foot.setRotation(q2);

    ros::Time t = ros::Time::now();
    //ros::Time(0)  ros::Time::now()
    tf_map_odom_br.sendTransform(tf::StampedTransform(tf_map_odom, ros::Time::now(), "map", "robot1_tf/odom"));
    //tf_odom_foot_br.sendTransform(tf::StampedTransform(tf_odom_foot, t, "robot1_tf/odom", "robot1_tf/base_footprint"));
}

int main(int argc, char** argv){
    ros::init(argc, argv,"PublishGroundTruth");

    ros::NodeHandle n;
    ros::Subscriber map_sub = n.subscribe("robot1/ground_truth/state", 5, &odom_callback);

    int hertz = 24;
    ros::Rate r(hertz);
    while(ros::ok()){
        ros::spinOnce();

        r.sleep();
    }

    return 0;
}