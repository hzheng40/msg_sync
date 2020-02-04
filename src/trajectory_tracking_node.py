#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import *

import numpy as np

import os
home_dir = os.environ['HOME']
import sys
sys.path.append(home_dir + '/formulazero/Simulator/python')
from mpc import pure_pursuit_utils


class TrajectoryTracker(object):
    def __init__(self):
        # get params
        self.traj_topic = rospy.get_param('picked_traj_topic')
        self.pose_topic = rospy.get_param('pf_odom_topic')
        self.drive_topic = rospy.get_param('drive_topic')
        self.num_states = rospy.get_param('num_states')
        self.start_boost_butt = rospy.get_param('starting_boost_button')
        self.joy_topic = rospy.get_param('joy_topic')

        # keeping track
        # NOTE that None in trajectory should be handled on the traj bridge side, if picked traj is None, the tracker will keep tracking the latest available picked traj
        self.latest_traj = None
        # tracking param (lookahead)
        self.track_lad = 1.0
        self.wheelbase = 0.3302
        self.max_reacquire = 10.
        # starting param
        self.start_counter = 0
        self.start_counter_thresh = 1
        self.start_button_pressed = False
        # ROS pub subs
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)
        self.traj_sub = rospy.Subscriber(self.traj_topic, Float64MultiArray, self.traj_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.pose_topic, Odometry, self.pose_callback, queue_size=1)
        self.joy_sub = rospy.Subscriber(self.joy_topic, Joy, self.joy_callback, queue_size=1)
        
    def pub_drive(self, speed, steer):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steer
        self.drive_pub.publish(drive_msg)

    def joy_callback(self, joy_msg):
        # if button pressed and below start step threshold, accel
        # else do nothing
        if joy_msg.buttons[self.start_boost_butt] and not self.start_button_pressed:
            rospy.loginfo('Button flipped')
            self.start_button_pressed = True
        while (self.start_counter < self.start_counter_thresh and self.start_button_pressed):
            rospy.loginfo('Starting...')
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 8.0
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            self.start_counter += 1
        pass

    def traj_callback(self, traj_msg):
        traj_arr = np.array(traj_msg.data)
        self.latest_traj = traj_arr.reshape((self.num_states, 4))
        
    def pose_callback(self, pf_odom_msg):
        if self.latest_traj is None:
            return
        pf_pose = pf_odom_msg.pose.pose
        _, _, yaw = euler_from_quaternion([pf_pose.orientation.x, pf_pose.orientation.y, pf_pose.orientation.z, pf_pose.orientation.w])
        # pose = [pf_pose.position.x, pf_pose.position.y, yaw]
        # pose_x, pose_y, pose_theta = pose
        next_speed, next_steer = self.pure_pursuit(pf_pose.position.x, pf_pose.position.y, yaw, self.latest_traj, self.track_lad)
        self.pub_drive(next_speed, next_steer)

    def pure_pursuit(self, pose_x, pose_y, pose_theta, trajectory, lookahead_distance):
        position = np.array([pose_x, pose_y])
        lookahead_point = self.get_current_waypoint(trajectory, lookahead_distance, position, pose_theta)
        if lookahead_point is None:
            # no lookahead point found, meaning that even the latest trajectory is tracked to the end, should just stop
            return 0.0, 0.0
        speed, steering_angle = pure_pursuit_utils.get_actuation(pose_theta, lookahead_point, position, lookahead_distance, self.wheelbase)
        # NOTE that clipping needed here because we're on the real car
        if abs(steering_angle) > 0.43:
            steering_angle = (steering_angle/abs(steering_angle))*0.43
        return speed, steering_angle

    def get_current_waypoint(self, waypoints, lookahead_distance, position, theta):
        wpts = waypoints[:, 0:2]
        nearest_point, nearest_dist, t, i = pure_pursuit_utils.nearest_point_on_trajectory_py2(position, wpts)
        if nearest_dist < lookahead_distance:
            lookahead_point, i2, t2 = pure_pursuit_utils.first_point_on_trajectory_intersecting_circle(position, lookahead_distance, wpts, i+t, wrap=True)
            if i2 == None:
                return None
            current_waypoint = np.empty(waypoints[i2, :].shape)
            # x, y
            current_waypoint[0:2] = waypoints[i2, 0:2]
            # theta
            current_waypoint[3] = waypoints[i2, 3]
            # speed
            current_waypoint[2] = waypoints[i2, 2]
            return current_waypoint
        elif nearest_dist < self.max_reacquire:
            return waypoints[i, :]
        else:
            return None

if __name__ == "__main__":
    rospy.init_node('traj_tracker')
    tracker = TrajectoryTracker()
    rospy.spin()