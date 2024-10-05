#!/usr/bin/env python

import math
import geometry_msgs.msg
from ur3_leader_follower.msg import FollowWaypointsResult

class Sequence(object):
    def __init__(self, seq_type):
        self.type = seq_type
        self.min_segment_length = 0.006
        self.delay = 0


class LineSequence(Sequence):
    def __init__(self, dx, dy, dz):
        super(LineSequence, self).__init__("line")

        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.final_pose = geometry_msgs.msg.Pose()
    
    def get_initial_waypoints(self, start_pose):
        self.final_pose.position.x = start_pose.position.x + self.dx
        self.final_pose.position.y = start_pose.position.y + self.dy
        self.final_pose.position.z = start_pose.position.z + self.dz
        self.final_pose.orientation = start_pose.orientation

        return self.generate_waypoints()
    
    def generate_waypoints(self):       
        return [self.final_pose]
    

class CircleSequence(Sequence):
    tau = 2*math.pi
    
    def __init__(self, radius, direction, revolutions):
        super(CircleSequence, self).__init__("circle")

        self.radius = radius
        self.direction = direction
        self.revolutions = revolutions
                
        self.xc = None
        self.yc = None
        self.zc = None
        self.orientation = None
        self.theta_c = 0
        self.completed_revolutions = 0
        
    def get_initial_waypoints(self, start_pose):
        self.xc = start_pose.position.x
        self.yc = start_pose.position.y
        self.zc = start_pose.position.z - self.radius
        self.orientation = start_pose.orientation

        return self.generate_waypoints()

    def generate_waypoints(self):
        if self.direction == "cw":
            total_radians = self.theta_c + self.tau*(self.revolutions-self.completed_revolutions-1)
        else:
            total_radians = self.tau-self.theta_c + self.tau*(self.revolutions-self.completed_revolutions-1)    
        total_dist = total_radians * self.radius
        num_segments = int(total_dist//self.min_segment_length)
        d_theta = total_radians/num_segments

        if self.direction == "cw":
            d_theta *= -1
            
        theta = self.theta_c
        waypoints = []
        for i in range(num_segments):
            theta += d_theta

            pose = geometry_msgs.msg.Pose()
            pose.position.x = self.xc
            pose.position.y = self.yc + (self.radius * math.sin(theta))
            pose.position.z = self.zc + (self.radius * math.cos(theta))
            pose.orientation = self.orientation

            waypoints.append(pose)

        return waypoints

class TriangleSequence(Sequence):
    def __init__(self, side_length, direction, revolutions):
        super(TriangleSequence, self).__init__("triangle")

        self.side_length = side_length
        self.direction = direction
        self.revolutions = revolutions
        self.side_name = None
        if self.direction == 'cw':
            self.side_name = FollowWaypointsResult.SIDEC
        elif self.direction == 'ccw':
            self.side_name = FollowWaypointsResult.SIDEA
        self.completed_revolutions = 0
        self.top = geometry_msgs.msg.Pose() 
        self.left = geometry_msgs.msg.Pose() 
        self.right = geometry_msgs.msg.Pose()   
        self.corners = {'cw': [],
                        'ccw': []}
        
    def get_initial_waypoints(self, start_pose):
        """ Generates waypoints for an equilateral triangle in the plane of the bar with start_pose being the top of
        the triangle"""
        
        self.top = start_pose
        
        self.right.position.x = self.top.position.x
        self.right.position.y = self.top.position.y - self.side_length * math.cos(math.radians(60))
        self.right.position.z = self.top.position.z - self.side_length * math.sin(math.radians(60))
        self.right.orientation = self.top.orientation

        self.left.position.x = self.top.position.x
        self.left.position.y = self.top.position.y + self.side_length * math.cos(math.radians(60))
        self.left.position.z = self.top.position.z - self.side_length * math.sin(math.radians(60))
        self.left.orientation = self.top.orientation
        self.corners['cw'] = [self.right, self.left, self.top]
        self.corners['ccw'] = [self.left ,self.right, self.top]
        
        return self.generate_waypoints()
    
    def generate_waypoints(self):
        waypoints = []
        c_remaining = None
        
        if self.direction == 'cw':
            c_remaining = self.side_name + 1
        elif self.direction == 'ccw':
            c_remaining = 3 - self.side_name
            
        for c in range(c_remaining):
            waypoints.append(self.corners[self.direction][c+len(self.corners[self.direction])-c_remaining])
        
        for _ in range(self.revolutions-self.completed_revolutions-1):
            for corner in self.corners[self.direction]:
                waypoints.append(corner) 
       
        return waypoints