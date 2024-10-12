#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Vector3, Quaternion, TwistStamped
import math

class SatelliteController:
    def __init__(self):
        rospy.init_node('satellite_controller', anonymous=True)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Initialize the publisher for the velocity
        self.velocity_pub = rospy.Publisher('/chaser/velocity', TwistStamped, queue_size=10)

        # Satellite parameters and state initialization
        self.radius = Vector3(678.2, 670.2, 900)  # Initial orbital radius
        self.orientation = Quaternion(0, 1.57, 1, 0)  # Initial orientation
        self.current_orbit_speed = 0.0005  # Initial orbital speed in rad/s
        self.target_orbit_speed = self.current_orbit_speed

        self.update_radius = self.radius
        self.update_orientation = self.orientation
        self.transition_duration = 5000.0
        self.transition_start_time = None

        # Subscribe to trajectory updates
        rospy.Subscriber('/satellite/trajectory', ModelState, self.update_trajectory)
        self.rate = rospy.Rate(10)

    def update_trajectory(self, msg):
        rospy.loginfo("Received new trajectory update...")
        self.update_radius = msg.pose.position
        self.update_orientation = msg.pose.orientation
        self.target_orbit_speed = msg.twist.linear.x
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
                
                # Update radius and speed based on the interpolation
                self.radius.x = self.interpolate(self.radius.x, self.update_radius.x, elapsed)
                self.current_orbit_speed = self.interpolate(self.current_orbit_speed, self.target_orbit_speed, elapsed)
                self.orientation = Quaternion(
                    x=self.interpolate(self.orientation.x, self.update_orientation.x, elapsed),
                    y=self.interpolate(self.orientation.y, self.update_orientation.y, elapsed),
                    z=self.interpolate(self.orientation.z, self.update_orientation.z, elapsed),
                    w=self.interpolate(self.orientation.w, self.update_orientation.w, elapsed)
                )

            # Calculate new position based on updated radius and speed
            angle += self.current_orbit_speed * elapsed_time
            angle %= 2 * math.pi  # Normalize the angle
            x = self.radius.x * math.cos(angle)
            y = self.radius.y * math.sin(angle)
            z = self.radius.z

            # Publish the updated state to Gazebo
            state_msg = ModelState()
            state_msg.model_name = 'chief'
            state_msg.pose.position = Vector3(x, y, z)
            state_msg.pose.orientation = self.orientation
            try:
                self.set_state_service(state_msg)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

            # Publish the velocity
            velocity_msg = TwistStamped()
            velocity_msg.twist.linear.x = self.current_orbit_speed * math.cos(angle)
            velocity_msg.twist.linear.y = self.current_orbit_speed * math.sin(angle)
            velocity_msg.twist.linear.z = 0  # Assuming no z-component for simplicity
            self.velocity_pub.publish(velocity_msg)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = SatelliteController()
        controller.move_satellite()
    except rospy.ROSInterruptException:
        pass