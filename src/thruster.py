#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3, Quaternion, Twist
from gazebo_msgs.msg import ModelState

def publish_trajectory_once():
    rospy.init_node('trajectory_publisher', anonymous=True)
    trajectory_pub = rospy.Publisher('/satellite/trajectory', ModelState, queue_size=10)

    rospy.sleep(2)  # Wait for the publisher to be ready

    # Create a ModelState message with position, orientation, and intended velocities
    trajectory = ModelState()
    trajectory.model_name = 'chief'
    trajectory.pose.position = Vector3(679.2, 679.2, 900)  # Desired position
    trajectory.pose.orientation = Quaternion(0, 0, 0, 1)  # Desired orientation
    trajectory.twist.linear = Vector3(0.0005, 1, 0)  # Set desired velocities (as an example)

    trajectory_pub.publish(trajectory)

    rospy.sleep(2)  # Keep node alive to ensure message is sent

if __name__ == '__main__':
    try:
        publish_trajectory_once()
    except rospy.ROSInterruptException:
        pass
