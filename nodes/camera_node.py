#!/usr/bin/env python

import rospy
import sys
from ur3_leader_follower.camera import Camera

if __name__ == "__main__":
    rospy.init_node('camera_node', xmlrpc_port=53000, tcpros_port=53001)
    
    camera = Camera()
    rospy.on_shutdown(camera.shutdown)

    camera.read_frames()
