#!/usr/bin/env python

import rospy
import math
import tf2_ros
import tf.transformations
import geometry_msgs.msg


class BarStateBroadcaster:
    def __init__(self):
        self.br = tf2_ros.StaticTransformBroadcaster()

        bar_initial_pose = rospy.get_param("bar_initial_pose")
        bar_length = rospy.get_param("bar_length")
        grip_approach_offset = rospy.get_param("grip_approach_offset")
        grip_start_offset = rospy.get_param("grip_start_offset")

        bar_preload = rospy.get_param("bar_preload")
        ft_zero_offset = rospy.get_param("ft_zero_offset")

        self.static_transforms = []

        self.static_transforms.append(self.generate_transform("world", "bar_stand",
                                                              [bar_initial_pose["x"],
                                                               bar_initial_pose["y"],
                                                               bar_initial_pose["z"]],
                                                              [bar_initial_pose["roll"],
                                                               bar_initial_pose["pitch"],
                                                               bar_initial_pose["yaw"]]))

        self.static_transforms.append(self.generate_transform("bar_stand", "leader_grip",
                                                              [bar_length/2, 0, 0],
                                                              [0, -math.pi/2, 0]))

        self.static_transforms.append(self.generate_transform("bar_stand", "follower_grip",
                                                              [-bar_length/2, 0, 0],
                                                              [math.pi, -math.pi/2, 0]))

        self.static_transforms.append(self.generate_transform("leader_grip", "leader_approach",
                                                              [0, 0, -grip_approach_offset],
                                                              [0, 0, 0]))
        self.static_transforms.append(self.generate_transform("follower_grip", "follower_approach",
                                                              [0, 0, -grip_approach_offset],
                                                              [0, 0, 0]))
        
        self.static_transforms.append(self.generate_transform("leader_grip", "leader_preload",
                                                              [0, 0, -bar_preload/2],
                                                              [0, 0, 0]))
        
        self.static_transforms.append(self.generate_transform("follower_grip", "follower_preload",
                                                              [0, 0, -bar_preload/2],
                                                              [0, 0, 0]))
        
        self.static_transforms.append(self.generate_transform("leader_preload", "leader_ft_bias",
                                                              [ft_zero_offset, 0, 0],
                                                              [0, 0, 0]))
        
        self.static_transforms.append(self.generate_transform("follower_preload", "follower_ft_bias",
                                                              [ft_zero_offset, 0, 0],
                                                              [0, 0, 0]))

        self.static_transforms.append(self.generate_transform("leader_approach", "leader_start",
                                                              [grip_start_offset, 0, 0],
                                                              [0, 0, 0]))
        self.static_transforms.append(self.generate_transform("follower_approach", "follower_start",
                                                              [grip_start_offset, 0, 0],
                                                              [0, 0, 0]))

        self.static_transforms.append(self.generate_transform("leader_tcp", "bar_center",
                                                              [0, 0, bar_length/2 + bar_preload/2],
                                                              [0, math.pi/2, 0]))

        self.static_transforms.append(self.generate_transform("bar_center", "follower_attach",
                                                              [-bar_length/2 - bar_preload/2, 0, 0],
                                                              [math.pi, -math.pi/2, 0]))

        self.static_transforms.append(self.generate_transform("bar_center", "leader_attach",
                                                              [bar_length/2 + bar_preload/2, 0, 0],
                                                              [0, -math.pi/2, 0]))

    @staticmethod
    def generate_transform(parent, child, position, orientation):
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()

        x, y, z = position
        roll, pitch, yaw = orientation
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        return t

if __name__ == "__main__":
    rospy.init_node('bar_state_publisher')
    broadcaster = BarStateBroadcaster()
    broadcaster.br.sendTransform(broadcaster.static_transforms)
    rospy.spin()
