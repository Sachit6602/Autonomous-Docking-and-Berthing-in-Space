#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Vector3, Quaternion
import math

class SatelliteController:
    def __init__(self):
        rospy.init_node('satellite_controller', anonymous=True)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.radius = Vector3(680.2, 680.2, 900)  # Assume initial radius in the x direction
        self.orientation = Quaternion(0, 0, 1, 0)  # Neutral orientation
        self.current_orbit_speed = 0.0005  # Initial rad/s
        self.target_orbit_speed = self.current_orbit_speed  # Target speed

        self.update_radius = self.radius
        self.update_orientation = self.orientation
        self.transition_duration = 500.0
        self.transition_start_time = None


        self.rate = rospy.Rate(10)

    def update_trajectory(self, msg):
        rospy.loginfo("Received new trajectory update...")
        self.update_radius = msg.pose.position
        self.update_orientation = msg.pose.orientation
        self.target_orbit_speed = msg.twist.linear.x  # Assume linear.x holds target speed
        self.transition_start_time = rospy.get_time()
        self.transition_duration = msg.twist.linear.y if msg.twist.linear.y != 0 else 100.0

    def interpolate(self, start, end, factor):
        return start + (end - start) * factor

    def move_satellite(self):
        last_time = rospy.get_time()
        angle = 0.0

        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            elapsed_time = current_time - last_time
            last_time = current_time

            if self.transition_start_time:
                elapsed = (current_time - self.transition_start_time) / self.transition_duration
                elapsed = min(elapsed, 1.0)
                
                self.radius.x = self.interpolate(self.radius.x, self.update_radius.x, elapsed)
                self.current_orbit_speed = self.interpolate(self.current_orbit_speed, self.target_orbit_speed, elapsed)
                self.orientation = Quaternion(
                    x=self.interpolate(self.orientation.x, self.update_orientation.x, elapsed),
                    y=self.interpolate(self.orientation.y, self.update_orientation.y, elapsed),
                    z=self.interpolate(self.orientation.z, self.update_orientation.z, elapsed),
                    w=self.interpolate(self.orientation.w, self.update_orientation.w, elapsed)
                )

            angle += self.current_orbit_speed * elapsed_time
            angle %= 2 * math.pi  # Normalize the angle

            x = self.radius.x * math.cos(angle)
            y = self.radius.y * math.sin(angle)
            z = self.radius.z

            state_msg = ModelState()
            state_msg.model_name = 'deputy'
            state_msg.pose.position = Vector3(x, y, z)
            state_msg.pose.orientation = self.orientation

            try:
                self.set_state_service(state_msg)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = SatelliteController()
        controller.move_satellite()
    except rospy.ROSInterruptException:
        pass
