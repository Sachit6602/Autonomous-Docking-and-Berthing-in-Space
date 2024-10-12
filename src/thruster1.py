#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3, Quaternion, Twist
from gazebo_msgs.msg import ModelState

def publish_trajectory(speed_factor, duration):
    rospy.init_node('trajectory_publisher', anonymous=True)
    trajectory_pub = rospy.Publisher('/satellite/trajectory', ModelState, queue_size=10)

    # Wait for the publisher to be ready
    rospy.sleep(1)  # Increased delay

    # Create a ModelState message with position, orientation, and velocity
    trajectory = ModelState()
    trajectory.model_name = 'chief'
    trajectory.pose.position = Vector3(678.2, 678.2, 900)  # Default radius
    trajectory.pose.orientation = Quaternion(0, 0, 0, 1)   # Default orientation

    # Using twist to communicate speed factor
    trajectory.twist.linear.x = speed_factor
    trajectory.twist.linear.y = duration

    rospy.loginfo(f"Publishing new trajectory: Speed Factor={speed_factor}, Duration={duration}")
    trajectory_pub.publish(trajectory)

    # Keep the node alive for a short duration to ensure the message is sent
    rospy.sleep(2)

if __name__ == '__main__':
    try:
        # Speed up for 10 seconds
        speed_factor = 0.0005# Example speed factor (2x faster)
        duration = 0.1       # Example duration (10 seconds)
        publish_trajectory(speed_factor, duration)
        
        rospy.sleep(0.1)  # Wait a bit before sending the next command
        
        # Reset to normal speed
        speed_factor = 0.0005  # Normal speed
        duration = 0        # No duration needed for resetting
        publish_trajectory(speed_factor, duration)
    except rospy.ROSInterruptException:
        pass
