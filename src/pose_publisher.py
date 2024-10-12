#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped

class TargetPosePublisher:
    def __init__(self):
        rospy.init_node('target_pose_publisher', anonymous=True)
        self.pose_pub = rospy.Publisher('/target/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.target_name = 'deputy'  # Name of the target model in Gazebo
        self.rate = rospy.Rate(10)  # 10 Hz

    def model_states_callback(self, msg):
        try:
            index = msg.name.index(self.target_name)
            target_pose = msg.pose[index]

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = 'world'
            pose_msg.pose = target_pose

            self.pose_pub.publish(pose_msg)
        except ValueError:
            rospy.logwarn("Target model not found in /gazebo/model_states")

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        target_pose_publisher = TargetPosePublisher()
        target_pose_publisher.spin()
    except rospy.ROSInterruptException:
        pass