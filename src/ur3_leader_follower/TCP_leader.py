#!/usr/bin/env python

import rospy
import math
import actionlib
import actionlib_msgs.msg
import geometry_msgs.msg
import PyKDL
import threading
import tf_conversions.posemath as pm
from control_msgs.msg import FollowJointTrajectoryGoal
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal, FollowCartesianTrajectoryResult, CartesianTrajectory, CartesianTrajectoryPoint
from ur3_leader_follower.robot import Robot
from ur3_leader_follower.msg import FollowWaypointsAction, FollowWaypointsFeedback, FollowWaypointsResult
from ur3_leader_follower.msg import Sequence
from std_msgs.msg import Float32
import numpy as np

class Leader(Robot):
    def __init__(self):
        # Inherit everything from Robot class
        super(Leader, self).__init__("leader_")

        self.gripper_pin = 4
        self.home_joint_state = [1.5708, -1.5708, 1.5708, 0, 1.5708, 4.71239]
        self.ee_speed = float(rospy.get_param('ee_speed'))
        
        if self.ee_speed > 0.3:
            rospy.logerr("Leader end effector speed cannot be greater than 0.3 m/s, resetting to 0.1 m/s")
            self.ee_speed = 0.1
        
        self.follow_waypoints_server = actionlib.SimpleActionServer('follow_waypoints', FollowWaypointsAction,
                                                                    self.follow_waypoints, auto_start=False)

        # Define member variables
        self.current_sequence_type = None 
        self.radius = 0.08
        self.angle_offset = [float(rospy.get_param("angle_offset"))]
        self.feedback = bool(rospy.get_param("feedback"))
        self.filtered_angle = 0.0
        
        # Used for filtered angle calc
        self.alpha = 0.05
        
        # Create a transform from leader_tcp to follower_attach
        t = self.tfBuffer.lookup_transform("leader_tcp", "follower_attach", rospy.Time(), rospy.Duration(1)).transform
        self.leader_tcp_to_follower_attach_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                                                               PyKDL.Vector(t.translation.x, t.translation.y, t.translation.z))
        
        self.follower_attach_pose = geometry_msgs.msg.PoseStamped()
        self.follower_attach_pose.header.frame_id = 'world'
        
        # Set a timer to publish the desired follower attach pose every 8 ms (125 Hz)
        # Default case with no correction algorithm
        if (self.feedback == False and self.angle_offset == 0):
            self.desired_follower_tcp_pose_pub = rospy.Publisher("/follower/desired_tcp_pose",
                                                             geometry_msgs.msg.PoseStamped,
                                                             queue_size=1, tcp_nodelay=True)
            self.t1 = threading.Thread(target=rospy.Timer(rospy.Duration.from_sec(0.008), self.desired_pose_pub_callback))   
        # Correction algorithm on with error feedback from follower
        elif (self.feedback == True):
            self.t2 = threading.Thread(target=rospy.Subscriber("real_time_cartesian_error", Float32 , self.update_angle_offset))
            self.desired_follower_tcp_pose_pub = rospy.Publisher("/follower/desired_tcp_pose",
                                                             geometry_msgs.msg.PoseStamped,
                                                             queue_size=1, tcp_nodelay=True)
            self.t1 = threading.Thread(target=rospy.Timer(rospy.Duration.from_sec(0.008), self.desired_pose_pub_callback_feedback))
            self.t2.start()
        # Constant angle offset without feedback       
        else:    
            self.t1 = threading.Thread(target=rospy.Timer(rospy.Duration.from_sec(0.008), self.desired_pose_pub_callback_feedback))
        self.t1.start()
        
        rospy.on_shutdown(self.cleanup)

    def update_angle_offset(self, msg):
        #print("Received {}".format(msg))
        #convert cartesian error to angular offset
        self.angle_offset[0] = math.acos((2*(self.radius ** 2) - msg.data ** 2)
                                             /(2*(self.radius**2)))

    def follow_waypoints(self, goal):
        result = FollowWaypointsResult()
        feedback = FollowWaypointsFeedback()
        
        # Switch Controllers
        if not self.switch_controller("pose_based_cartesian_traj_controller"):
            self.follow_waypoints_server.set_aborted()
            return

        # Create goal for action client
        traj_goal = FollowCartesianTrajectoryGoal()
        traj_goal.trajectory = self.generate_cartesian_trajectory(goal.waypoints)

        feedback.ready_to_move = True
        self.follow_waypoints_server.publish_feedback(feedback)

        # Wait for sequence trigger
        while not rospy.get_param("/start_sequence"):
            pass

        self.cart_trajectory_client.send_goal(traj_goal)

        # Wait until the goal status is canceled or succeeded
        theta_p = None
        result.completed_rev = 0
        start_time = rospy.get_rostime()
        rate = rospy.Rate(125)
        while not rospy.is_shutdown(): 
            # Check if supervisor requested a preempt
            self.current_sequence_type = goal.sequence.type
            if self.follow_waypoints_server.is_preempt_requested():
                self.cart_trajectory_client.cancel_goal()
                result.success = False
                self.follow_waypoints_server.set_aborted(result)
                return
            
            # Check if action client reported as aborted
            state = self.cart_trajectory_client.get_state()
            if state == actionlib.GoalStatus.ABORTED or state == actionlib.GoalStatus.PREEMPTED:
                rospy.logerr(self.cart_trajectory_client.get_goal_status_text())
                rospy.logerr("The leader was unable to complete the requested trajectory")
                result.success = False
                self.follow_waypoints_server.set_aborted(result)
                return

            # Check if action client reported as succeeded
            if state == actionlib.GoalStatus.SUCCEEDED:
                result.success = True
                self.follow_waypoints_server.set_succeeded(result)
                return
            
            # Calculating current theta_c and completed revolutions to result
            if goal.sequence.type == Sequence.CIRCLE:
                self.circle_yc = goal.sequence.circle_yc
                self.circle_zc = goal.sequence.circle_zc
                current_pos = self.get_tcp_pose('msg')
                dy = current_pos.position.y - goal.sequence.circle_yc
                dz = current_pos.position.z - goal.sequence.circle_zc  
                result.theta_c = math.atan2(dy,dz)
                
                if result.theta_c < 0:
                    result.theta_c += 2*math.pi
                
                time_from_start = rospy.get_rostime() - start_time
                if goal.first_attempt and time_from_start.to_sec() < 1:
                    continue
                    
            elif goal.sequence.type == Sequence.TRIANGLE:
                q = self.get_tcp_pose('msg').position
                yc = goal.sequence.triangle_yt
                zc = goal.sequence.triangle_zt - (math.sqrt(7/3.0)*goal.sequence.tiangle_side_length)/2.0
                vector_tc = np.array([0,goal.sequence.triangle_zt - zc])
                vector_qc = np.array([q.y - yc, q.z - zc])
                result.theta_c = math.acos(np.dot(vector_tc,vector_qc)/(np.linalg.norm(vector_tc)*np.linalg.norm(vector_qc)))
                if q.y > yc:
                    result.theta_c = 2*math.pi - result.theta_c
                             
                if 0 <= result.theta_c < math.radians(120):
                    result.side_name = FollowWaypointsResult.SIDEC
                elif math.radians(120) <= result.theta_c < math.radians(240):
                    result.side_name = FollowWaypointsResult.SIDEB
                else:
                    result.side_name = FollowWaypointsResult.SIDEA
                
                time_from_start = rospy.get_rostime() - start_time
                if goal.first_attempt and time_from_start.to_sec() < 1:
                    continue
                
            if goal.sequence.type == Sequence.CIRCLE or goal.sequence.type == Sequence.TRIANGLE:
                if not theta_p:
                    theta_p = result.theta_c
                elif abs(result.theta_c - theta_p) >= math.pi:
                    result.completed_rev += 1
                    theta_p = result.theta_c
                else:
                    theta_p = result.theta_c
                    
            rate.sleep()   
            
    # Sending positional updates to follower with correction        
    def desired_pose_pub_callback_feedback(self, event):
        #rospy.loginfo("Request")
        self.follower_attach_pose.header.stamp = rospy.get_rostime()

        # Calculate the tcp_pose in world frame
        leader_tcp_frame = self.get_tcp_pose('frame')
        
        # Multiply the static transform from leader tcp to the follower attach frame
        follower_attach_frame = leader_tcp_frame * self.leader_tcp_to_follower_attach_frame

        # Only apply correction during circle sequence
        if (self.current_sequence_type == Sequence.CIRCLE):
            dy = follower_attach_frame.p.y() - self.circle_yc
            dz = follower_attach_frame.p.z() - self.circle_zc
            theta_c = math.atan2(dy,dz)
            #print("Angle offset: {}".format(self.angle_offset[0]))
            
            # Calculate the filtered angle and apply it to current leader theta
            self.filtered_angle = self.alpha * self.angle_offset[0] + (1 - self.alpha) * self.filtered_angle
            theta_c -= self.filtered_angle
            
            #print("filtered angle offset: {}".format(self.filtered_angle))
            if theta_c < 0:
                theta_c += 2*math.pi
                
            # Find new points     
            new_z = self.circle_zc + self.radius * math.cos(theta_c)
            new_y = self.circle_yc + self.radius * math.sin(theta_c)		
            follower_attach_frame.p = PyKDL.Vector(follower_attach_frame.p.x(), new_y, new_z)
            
        self.follower_attach_pose.pose = pm.toMsg(follower_attach_frame)
        
        self.desired_follower_tcp_pose_pub.publish(self.follower_attach_pose)
        #rospy.loginfo("Sent")
        
    # Default way of sending, without correction and feedback
    def desired_pose_pub_callback(self, event):
        #rospy.loginfo("Request")
        self.follower_attach_pose.header.stamp = rospy.get_rostime()

        # Calculate the tcp_pose in world frame
        leader_tcp_frame = self.get_tcp_pose('frame')
        
        # Multiply the static transform from leader tcp to the follower attach frame
        follower_attach_frame = leader_tcp_frame * self.leader_tcp_to_follower_attach_frame
            
        self.follower_attach_pose.pose = pm.toMsg(follower_attach_frame)
        
        self.desired_follower_tcp_pose_pub.publish(self.follower_attach_pose)
            
    def cleanup(self):
        self.desired_pose_pub_timer.shutdown()
