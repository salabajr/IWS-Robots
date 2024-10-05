#!/usr/bin/env python

import rospy
from ur3_leader_follower.leader import Leader

if __name__ == "__main__":
    rospy.init_node('leader_node', xmlrpc_port=51000, tcpros_port=51001)
    leader = Leader()

    leader.follow_waypoints_server.start()
    rospy.loginfo("Leader robot is ready to receive action goals")

    rospy.spin()
