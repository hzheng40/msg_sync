#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
# from tf.transformations import *

import numpy as np
import argparse
import msgpack
import zmq

class TrajBridge(object):
    def __init__(self):
        # get params
        self.subport = rospy.get_param('traj_bridge_subport')
        self.traj_topic = rospy.get_param('picked_traj_topic')
        self.visualize = rospy.get_param('visualize_trajectory')
        self.visualize_topic = rospy.get_param('visualize_trajectory_topic')
        # zmq stuff
        context = zmq.Context()
        # picked traj sub
        self.socket = context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.SUBSCRIBE, b'')
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect('tcp://localhost:%s' % self.subport)
        self.topic = 'traj'

        # messeage filter subs
        self.traj_pub = rospy.Publisher(self.traj_topic, Float64MultiArray, queue_size=1)
        if self.visualize:
            self.traj_viz_pub = rospy.Publisher(self.visualize_topic, Marker, queue_size=1)
            self.num_states = rospy.get_param('num_states')

    def pub_traj(self, message):
        string = self.socket.recv()
        traj_dict = msgpack.loads(message, encoding='utf-8')
        traj_arr = traj_dict['trajectory']
        # traj_arr = list(recv_array(self.socket))
        msg = Float64MultiArray()
        msg.data = traj_arr
        self.traj_pub.publish(msg)
        if self.visualize:
            marker = Marker()
            marker.header.frame_id = '/map'
            marker.type = Marker.SPHERE_LIST
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.g = 1.0
            traj_arr = np.array(traj_arr)
            traj = traj_arr.reshape((self.num_states, 4))
            for pt in traj:
                pt_msg = Point()
                pt_msg.x = pt[0]
                pt_msg.y = pt[1]
                marker.points.append(pt_msg)
            self.traj_viz_pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node('traj_bridge', disable_signals=True)
    traj_bridge = TrajBridge()
    while True:
        string = traj_bridge.socket.recv()
        traj_bridge.pub_traj(string)