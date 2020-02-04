#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import message_filters
from tf.transformations import *

import numpy as np
import argparse
import msgpack
import zmq

class SensorSync(object):
    def __init__(self):
        # get params
        self.pubport = rospy.get_param('sensor_obs_pubport')
        self.num_scan_obs = rospy.get_param('num_scan_obs')
        self.scan_topic = rospy.get_param('scan_topic')
        self.pf_topic = rospy.get_param('pf_odom_topic')
        self.vesc_topic = rospy.get_param('vesc_odom_topic')

        # zmq stuff
        context = zmq.Context()
        # synced message pub
        self.socket = context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.bind('tcp://*:%s' % self.pubport)
        self.topic = 'synced_msg'

        # messeage filter subs
        self.scan_sub = message_filters.Subscriber(self.scan_topic, LaserScan, queue_size=1)
        self.pf_odom_sub = message_filters.Subscriber(self.pf_topic, Odometry, queue_size=1)
        self.vesc_odom_sub = message_filters.Subscriber(self.vesc_topic, Odometry, queue_size=1)

        ts = message_filters.ApproximateTimeSynchronizer([self.scan_sub, self.pf_odom_sub, self.vesc_odom_sub], 1, 0.1, allow_headerless=True)
        ts.registerCallback(self.sync_callback)
        self.scan_inited = False

    def sync_callback(self, scan, pf_odom, vesc_odom):
        # scans could be subsampled here?
        full_ranges = np.array(scan.ranges)
        # if scan not initialzed, save the params
        if not self.scan_inited:
            self.scan_sub_idx = np.linspace(0, full_ranges.shape[0]-1, self.num_scan_obs).astype(int)
            self.scan_inited = True
        # subsample scan
        sub_ranges = np.take(full_ranges, self.scan_sub_idx)

        # pose can be extracted as x, y, z, theta
        _, _, yaw = euler_from_quaternion([pf_odom.pose.pose.orientation.x, pf_odom.pose.pose.orientation.y, pf_odom.pose.pose.orientation.z, pf_odom.pose.pose.orientation.w])

        # pose covariance is 36 long array, row major 6x6 covmat, params in order are (x, y, z, rot_x, rot_y, rot_z), rotations are fixed axis
        pose_covariance = list(pf_odom.pose.covariance)

        # speed from vesc odom
        speed = vesc_odom.twist.twist.linear.x
        
        sync_msg = dict(
            scan = list(sub_ranges),
            pose = [pf_odom.pose.pose.position.x, pf_odom.pose.pose.position.y, yaw],
            pose_covariance = pose_covariance,
            speed = speed
        )
        msg_dump = msgpack.dumps(sync_msg)
        self.socket.send(msg_dump)


if __name__ == "__main__":
    rospy.init_node('message_sync')
    sync = SensorSync()
    rospy.spin()