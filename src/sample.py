#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3, Quaternion, Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState

def get_chaser_position():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        chaser_state = get_state('chaser', '')
        return chaser_state.pose.position
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

def publish_trajectory_once():
    rospy.init_node('trajectory_publisher', anonymous=True)
    trajectory_pub = rospy.Publisher('/satellite/trajectory', ModelState, queue_size=10)

    rospy.sleep(2)  # Wait for the publisher to be ready

    chaser_position = get_chaser_position()
    if chaser_position is None:
        rospy.logerr("Failed to get chaser position.")
        return

    # Create a ModelState message with the x and y positions from the chaser and change z
    trajectory = ModelState()
    trajectory.model_name = 'chief'
    trajectory.pose.position = Vector3(678.2, 678.2, 900)  # Keep x, y same; change z
    trajectory.pose.orientation = Quaternion(0, 0, 0, 1)  # Desired orientation
    trajectory.twist.linear = Vector3(0.0005, 1, 0)  # Set desired velocities (as an example)

    trajectory_pub.publish(trajectory)

    rospy.sleep(2)  # Keep node alive to ensure message is sent

if __name__ == '__main__':
    try:
        publish_trajectory_once()
    except rospy.ROSInterruptException:
        pass
