#!/usr/bin/env python

import math
from os.path import exists
import yaml
import numpy as np
import rospy
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import tf_conversions.posemath as pm
import ur3_leader_follower.sequences as sequence
from ur3_leader_follower.msg import Sequence as SequenceMsg

class SequenceHandler:
    def __init__(self, bar_start_pose, bar_to_leader_attach_transform):
        """Class that handles all tasks for the Supervisor related to sequences
        """      
        self.sequences = []
        self.current_sequence = None
        self.sequence_num = None
        self.bar_initial_waypoints = []
        
        self.bar_start_pose = bar_start_pose
        
        # Transformation Matrix from Bar Center to Leader Attach
        self.t_bar_la = pm.toMatrix(tf2_geometry_msgs.transform_to_kdl(bar_to_leader_attach_transform))
        
    def read_sequences(self):
        path = rospy.get_param("sequences_path")
        
        print("Reading sequences from {f}".format(f=path))
                
        # Check to see if file exists
        if not exists(path):
            rospy.logerr("Sequences path is not valid")
            return False
        
        stream = file(path, 'r')
        data = yaml.safe_load(stream)
        for d in data['sequences']:
            if d['type'] == 'line':
                try:
                    seq = sequence.LineSequence(d['dx'], d['dy'], d['dz'])
                except KeyError:
                    rospy.logerr("Unable to parse sequence")
                    return False
                    
            elif d['type'] == 'circle':
                try:
                    seq = sequence.CircleSequence(d['radius'], d['direction'], d['revolutions'])
                except KeyError:
                    rospy.logerr("Unable to parse sequence")
                    return False

            elif d['type'] == 'triangle':
                try:
                    seq = sequence.TriangleSequence(d['side_length'], d['direction'], d['revolutions'])
                except KeyError:
                    rospy.logerr("Unable to parse sequence")
                    return False

            else:
                rospy.logerr("Sequence type not recognized")
            
            if d.has_key('delay'):
                seq.delay = d['delay']
            
            self.sequences.append(seq)
        
        # Compute waypoints for each sequence 
        start = self.bar_start_pose
        for seq in self.sequences:
            self.bar_initial_waypoints.append(seq.get_initial_waypoints(start))
            start = self.bar_initial_waypoints[-1][-1]
            
        # Check to see if final position of bar matches initial position of bar
        pt1 = self.bar_start_pose
        pt2 = self.bar_initial_waypoints[-1][-1]
        d = math.sqrt((pt1.position.x - pt2.position.x)**2 +
                      (pt1.position.y - pt2.position.y)**2 +
                      (pt1.position.z - pt2.position.z)**2)
        
        if d > 0.001:
            rospy.logerr("End position of sequences does not match start position")
            return False
        
        rospy.loginfo("Successfully read sequences")
        return True
        
    def has_next(self):
        if self.sequence_num is None:
            return True
        if self.sequence_num + 1 < len(self.sequences):
            return True
        return False
    
    def get_next_sequence(self):
        if not self.current_sequence:
            self.sequence_num = 0
        else:
            self.sequence_num += 1
            
        self.current_sequence = self.sequences[self.sequence_num]
        
        sequence = SequenceMsg()
        if self.current_sequence.type == "line":
            sequence.type = SequenceMsg.LINE
        elif self.current_sequence.type == "circle":
            sequence.type = SequenceMsg.CIRCLE
            sequence.circle_yc = self.current_sequence.yc
            sequence.circle_zc = self.current_sequence.zc
        elif self.current_sequence.type == "triangle":
            sequence.type = SequenceMsg.TRIANGLE
            sequence.triangle_yt = self.current_sequence.top.position.y
            sequence.triangle_zt = self.current_sequence.top.position.z
            sequence.tiangle_side_length = self.current_sequence.side_length
            
        return sequence
    
    def get_waypoints(self):
        bar_waypoints = self.current_sequence.generate_waypoints()
                
        leader_waypoints = []
        for waypoint in bar_waypoints:
            # convert waypoint pose to matrix
            t_w_bar = pm.toMatrix(pm.fromMsg(waypoint))

            # calculate leader_attach in world
            t_w_la = np.dot(t_w_bar, self.t_bar_la)

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "world"
            pose_stamped.pose = pm.toMsg(pm.fromMatrix(t_w_la))
            
            leader_waypoints.append(pose_stamped)

        return leader_waypoints

    def update_circle_progress(self, theta_c, completed_revolutions):
        self.current_sequence.theta_c = theta_c
        self.current_sequence.completed_revolutions += completed_revolutions        

    def update_triangle_progress(self, side_name, completed_rev):
        self.current_sequence.side_name = side_name
        self.current_sequence.completed_revolutions += completed_rev
        
    def get_delay(self):
        return self.current_sequence.delay
