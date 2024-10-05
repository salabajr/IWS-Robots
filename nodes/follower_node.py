#!/usr/bin/env python

import rospy
from ur3_leader_follower.follower import Follower

if __name__ == "__main__":
    rospy.init_node('follower_node', xmlrpc_port=52000, tcpros_port=52001)

    follower = Follower()
    follower.follow_frame_server.start()
    rospy.loginfo("Follower robot is ready to receive action goals")

    rospy.spin()
