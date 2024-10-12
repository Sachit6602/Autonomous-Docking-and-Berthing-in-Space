#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>


const double n = 0.001027; 

geometry_msgs::PoseStamped target_pose;
geometry_msgs::Twist cmd_vel;

void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    target_pose = *msg;
}

void controlLoop(const ros::TimerEvent&) {
    double x = target_pose.pose.position.x;
    double y = target_pose.pose.position.y;
    double z = target_pose.pose.position.z;

    // HCW equations to calculate relative velocity
    cmd_vel.linear.x = 2 * n * y;
    cmd_vel.linear.y = -2 * n * x;
    cmd_vel.linear.z = 0;

    // Publish the command velocity
    cmd_pub.publish(cmd_vel);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "hcw_controller");
    ros::NodeHandle nh;

    ros::Subscriber target_pose_sub = nh.subscribe("/target/pose", 10, targetPoseCallback);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/chaser/cmd_vel", 10);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), controlLoop);
    ros::spin();

    return 0;
}