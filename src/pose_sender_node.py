#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseStamped
from tf.transformations import *

import numpy as np
import argparse
import msgpack
import zmq
from zmq_array_utils import send_array

# parser = argparse.ArgumentParser()
# pubport for synced messages
# parser.add_argument('--pubport', type=int, required=False, default=8888)
# args = parser.parse_args()
# PUBPORT = 8888

# PF_TOPIC = '/pf/pose/odom'
# PF_TOPIC = '/odom'

class PoseSender(object):
    def __init__(self):
        self.pf_topic = rospy.get_param('pf_odom_topic')
        self.pose_pubport = rospy.get_param('self_pose_pubport')
        # zmq stuff
        context = zmq.Context()
        # synced message pub
        self.socket = context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.bind('tcp://*:%s' % self.pose_pubport)
        self.topic = 'pose'

        # messeage filter subs
        self.pf_odom_sub = rospy.Subscriber(self.pf_topic, Odometry, self.pose_callback, queue_size=1)

    def pose_callback(self, pf_odom):
        # pose can be extracted as x, y, z, theta
        _, _, yaw = euler_from_quaternion([pf_odom.pose.pose.orientation.x, pf_odom.pose.pose.orientation.y, pf_odom.pose.pose.orientation.z, pf_odom.pose.pose.orientation.w])
        # TODO: get only needed cov
        # pose_covariance = list(pf_odom.pose.covariance)
        
        pose_msg = dict(
            pose = [pf_odom.pose.pose.position.x, pf_odom.pose.pose.position.y, 0.0, yaw, pf_odom.twist.twist.linear.x],
        #     pose_covariance = pose_covariance
        )
        msg_dump = msgpack.dumps(pose_msg)
        self.socket.send(msg_dump)
        # pose_arr = np.array([pf_odom.pose.pose.position.x, pf_odom.pose.pose.position.y, yaw, pf_odom.twist.twist.linear.x])
        # send_array(self.socket, np.ascontiguousarray(pose_arr))

if __name__ == "__main__":
    rospy.init_node('pose_sender')
    sync = PoseSender()
    rospy.spin()