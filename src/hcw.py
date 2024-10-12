#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion, Twist
from gazebo_msgs.msg import ModelState
import numpy as np

class HCWDockingController:
    def __init__(self):
        rospy.init_node('hcw_docking_controller', anonymous=True)

        # Earth's gravitational parameter (m^3/s^2) for geostationary orbit
        self.mu = 3.986004418e14
        self.a = 42164e3  # Semi-major axis of geostationary orbit (m)
        self.n = np.sqrt(self.mu / self.a**3)  # Mean motion (rad/s)

        # Publishers and subscribers
        self.trajectory_pub = rospy.Publisher('/satellite/trajectory', ModelState, queue_size=10)
        rospy.Subscriber('chaser/pose', PoseStamped, self.chaser_pose_callback)
        rospy.Subscriber('target/pose', PoseStamped, self.target_pose_callback)

        self.chaser_pose = None
        self.target_pose = None
        self.rate = rospy.Rate(1)  # Update rate in Hz

    def chaser_pose_callback(self, msg):
        self.chaser_pose = msg.pose

    def target_pose_callback(self, msg):
        self.target_pose = msg.pose

    def publish_trajectory(self):
        if self.chaser_pose and self.target_pose:
            # Calculate relative position
            rel_position = Vector3(
                x=self.target_pose.position.x - self.chaser_pose.position.x,
                y=self.target_pose.position.y - self.chaser_pose.position.y,
                z=self.target_pose.position.z - self.chaser_pose.position.z
            )

            # Calculate HCW accelerations
            ax = 3 * self.n**2 * rel_position.x + 2 * self.n * rel_position.y
            ay = -2 * self.n * rel_position.x
            az = -self.n**2 * rel_position.z

            # Prepare the ModelState message
            trajectory = ModelState()
            trajectory.model_name = 'chaser'
            trajectory.pose = self.chaser_pose
            trajectory.twist.linear.x = ax
            trajectory.twist.linear.y = ay
            trajectory.twist.linear.z = az

            # Publishing the trajectory update
            self.trajectory_pub.publish(trajectory)
            rospy.loginfo("Published HCW-based trajectory update to /satellite/trajectory.")

    def run(self):
        while not rospy.is_shutdown():
            self.publish_trajectory()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        hcw_controller = HCWDockingController()
        hcw_controller.run()
    except rospy.ROSInterruptException:
        pass
