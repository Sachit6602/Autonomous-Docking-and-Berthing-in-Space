#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped
import subprocess
import os
import sys

class PIDController:
    def __init__(self, kp, ki, kd, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0
        self.last_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(min(output, self.max_output), -self.max_output)

class SatelliteDockingController:
    def __init__(self):
        rospy.init_node('satellite_docking_controller', anonymous=True)
        self.trajectory_pub = rospy.Publisher('/satellite/trajectory', ModelState, queue_size=10)
        
        self.chaser_pose = None
        self.target_pose = None
        self.pid_y = PIDController(kp=0.1, ki=0.01, kd=0.05, max_output=0.0015)
        rospy.Subscriber('chaser/pose', PoseStamped, self.chaser_pose_callback)
        rospy.Subscriber('target/pose', PoseStamped, self.target_pose_callback)

        self.rate = rospy.Rate(10)
        self.last_time = rospy.get_time()
        self.y_aligned = False

        # Run sample.py at the start
        rospy.loginfo("Executing z-axis and orientation correction.")
        subprocess.run(["python3", "./sample.py"], cwd=os.path.dirname(os.path.abspath(__file__)))

    def chaser_pose_callback(self, msg):
        self.chaser_pose = msg.pose

    def target_pose_callback(self, msg):
        self.target_pose = msg.pose

    def publish_trajectory_update(self):
        if self.chaser_pose and self.target_pose:
            current_time = rospy.get_time()
            dt = current_time - self.last_time
            self.last_time = current_time

            y_diff = self.target_pose.position.y - self.chaser_pose.position.y
            rospy.loginfo(f"Current y_diff: {y_diff:.4f}")

            if 0 <= y_diff <= 0.0100 and not self.y_aligned:
                self.y_aligned = True
                rospy.loginfo("Y-axis alignment achieved, publishing new trajectory.")
                trajectory = ModelState()
                trajectory.model_name = 'chief'
                trajectory.pose.position = Vector3(679.2, 679.2, 900)
                trajectory.pose.orientation = Quaternion(0, 0, 0, 1)
                trajectory.twist.linear = Vector3(0.0005, 1, 0)
                self.trajectory_pub.publish(trajectory)
            elif not self.y_aligned:
                speed_factor_y = self.pid_y.compute(y_diff, dt)
                rospy.loginfo(f"Adjusting speed to align Y-axis: y_diff={y_diff:.4f}")
                new_y = self.chaser_pose.position.y + speed_factor_y
                new_x = self.chaser_pose.position.x
                trajectory = ModelState()
                trajectory.model_name = 'chief'
                trajectory.pose.position = Vector3(new_x, new_y, self.chaser_pose.position.z)
                trajectory.pose.orientation = Quaternion(0, 0, 0, 1)
                trajectory.twist.linear.x = speed_factor_y
                self.trajectory_pub.publish(trajectory)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_trajectory_update()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = SatelliteDockingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
