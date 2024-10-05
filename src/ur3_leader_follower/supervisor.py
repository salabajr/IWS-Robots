#!/usr/bin/env python

import yaml
import math
import numpy as np
import rospy
import rospkg
import actionlib
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
import tf_conversions.posemath as pm
import tf_conversions
from std_srvs.srv import Trigger, TriggerRequest
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from netft_utils.srv import SetBias, SetBiasRequest
from ur3_leader_follower.sequence_handler import SequenceHandler
from ur3_leader_follower.msg import FollowWaypointsAction, FollowWaypointsGoal, FollowWaypointsFeedback
from ur3_leader_follower.msg import FollowFrameAction, FollowFrameGoal, FollowFrameFeedback, FollowWaypointsResult
from ur3_leader_follower.msg import Sequence as SequenceMsg
from ur3_leader_follower.sequences import Sequence
from ur3_leader_follower.srv import ActuateGripper, ActuateGripperRequest
from ur3_leader_follower.srv import GoToPose, GoToPoseRequest
from ur3_leader_follower.srv import StartRecording, StartRecordingRequest
from std_msgs.msg import Bool

class Supervisor:
    def __init__(self):
        # TF listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)

        # Action Clients
        self.leader_follow_waypoints_client = actionlib.SimpleActionClient('/leader/follow_waypoints', FollowWaypointsAction)
        self.leader_feedback = FollowWaypointsFeedback()
        self.follower_follow_frame_client = actionlib.SimpleActionClient('/follower/follow_frame', FollowFrameAction)
        self.follower_feedback = FollowFrameFeedback()
        
        # Sequence Handler
        bar_start_pose = pm.toMsg(tf2_geometry_msgs.transform_to_kdl(self.lookup_transform('bar_stand', 'world')))
        bar_start_pose.position.z += rospy.get_param('ft_zero_offset')
        leader_transform = self.lookup_transform('leader_attach', 'bar_center')
        self.sequence_handler = SequenceHandler(bar_start_pose, leader_transform)

        rospy.set_param("start_sequence", False)
        self.motion_detected = False
        self.motion_detected_subscriber = rospy.Subscriber("motion_detected", Bool, callback=self.motion_detected_cb)
        
        # Motion services
        self.start_motion = rospy.ServiceProxy('start_motion', Trigger)
        self.stop_motion = rospy.ServiceProxy('stop_motion', Trigger)
        
    def actuate_gripper(self, robot, state):
        request = ActuateGripperRequest()
        
        actuate_gripper = rospy.ServiceProxy(robot + '/actuate_gripper', ActuateGripper)
        
        if state == "ON":
            request.gripper_state = True
        elif state == "OFF":
            request.gripper_state = False
        else:
            rospy.logerr("Incorrect value for actuate gripper state")
            return

        response = actuate_gripper(request)

        if response.success:
            rospy.loginfo("Turned {state} {robot}'s gripper".format(robot=robot, state=state.lower()))
        else:
            rospy.logwarn("Unable to actuate {robot}'s gripper".format(robot=robot))

    def start_camera_recording(self, testname):
        request = StartRecordingRequest()
        request.testname = testname
        
        srv = '/start_recording'

        try:
            rospy.wait_for_service(srv, timeout=rospy.Duration(1))
        except rospy.ROSException:
            rospy.logwarn("Unable to connect to {name}".format(name=srv))
            return False

        start_recording = rospy.ServiceProxy(srv, StartRecording)

        try:
            response = start_recording(request)
        except rospy.ServiceException:
            rospy.logwarn("Service did not process request")
            return False

        if response.message:
            rospy.logwarn(response.msg)
        
        if response.success:
            rospy.loginfo("Started camera recording")
            return True
        return False

    def stop_camera_recording(self):
        request = TriggerRequest()
        
        srv = '/stop_recording'

        try:
            rospy.wait_for_service(srv, timeout=rospy.Duration(1))
        except rospy.ROSException:
            rospy.logwarn("Unable to connect to {name}".format(name=srv))
            return

        stop_recording = rospy.ServiceProxy(srv, Trigger)

        try:
            response = stop_recording(request)
        except rospy.ServiceException:
            rospy.logwarn("Service did not process request")
            return

        if response.message:
            rospy.logwarn(response.msg)
        
        if response.success:
            rospy.loginfo("Stopped camera recording")

    def send_home(self, robot):
        rospy.loginfo("Sending {robot} to home".format(robot=robot))
        
        send_to_home = rospy.ServiceProxy(robot + '/go_home', Trigger)
        
        response = send_to_home(TriggerRequest())

        return response.success

    def send_to_pose(self, robot, pose_name, time_from_start):
        transform = self.lookup_transform(robot + "_" + pose_name, 'world')

        request = GoToPoseRequest()
        request.goal_pose.header.frame_id = 'world'
        request.goal_pose.pose = pm.toMsg(tf2_geometry_msgs.transform_to_kdl(transform))
        request.time_from_start = rospy.Duration(time_from_start)

        go_to_pose = rospy.ServiceProxy(robot + "/go_to_pose", GoToPose)

        rospy.loginfo("Sending {robot} to {name}".format(robot=robot, name=pose_name))
        
        response = go_to_pose(request)
        
        return response.success
    
    def send_robots_to_grip_bar(self):
        poses = ["start", "approach", "grip"]
        timing = [1.5, 1.75, 1]
        for pose, time in zip(poses, timing):
            if not self.send_to_pose('leader', pose, time):
                return False
            if not self.send_to_pose('follower', pose, time):
                return False
        
        self.actuate_gripper("leader", "ON")
        self.actuate_gripper("follower", "ON")
        
        rospy.sleep(1)
        
        poses = ["preload", "ft_bias"]
        timing = [0.5, 0.5]
        for pose, time in zip(poses, timing):
            if not self.send_to_pose('leader', pose, time):
                return False
            if not self.send_to_pose('follower', pose, time):
                return False
        
        return True
    
    def return_bar_to_stand(self):
        poses = ["preload", "grip"]
        timing = [0.5, 0.5]
        for pose, time in zip(poses, timing):
            if not self.send_to_pose('leader', pose, time):
                return False
            if not self.send_to_pose('follower', pose, time):
                return False
        
        self.actuate_gripper("leader", "OFF")
        self.actuate_gripper("follower", "OFF")
        
        if not self.send_to_pose('leader', 'approach', 1):
            return False
    
        if not self.send_to_pose('follower', 'approach', 1):
            return False
        
        return True
    
    def execute_sequences(self):
        # Check to see if there is another sequence on the stack
        while self.sequence_handler.has_next():
            sequence = self.sequence_handler.get_next_sequence()
            sequence_completed = False
            first_attempt = True
            while not sequence_completed:
                # Initalize sequence parameters
                rospy.set_param("start_sequence", False)
                self.leader_feedback.ready_to_move = False
                self.follower_feedback.ready_to_move = False
                pause_requested = False
                
                # Generate and send action goals
                leader_goal = FollowWaypointsGoal()
                leader_goal.waypoints = self.sequence_handler.get_waypoints()
                leader_goal.sequence = sequence
                leader_goal.first_attempt = first_attempt
                self.leader_follow_waypoints_client.send_goal(leader_goal, feedback_cb=self.leader_feedback_cb)

                follower_goal = FollowFrameGoal()
                follower_goal.frame_id = "follower_attach"
                self.follower_follow_frame_client.send_goal(follower_goal, feedback_cb=self.follower_feedback_cb)
                
                # Wait for leader and follower to be ready to move
                while not rospy.is_shutdown():
                    #print("leader ready to move: {}, follower ready to move: {}".format(self.leader_feedback.ready_to_move, self.follower_feedback.ready_to_move))
                    if self.leader_feedback.ready_to_move and self.follower_feedback.ready_to_move:
                        #print("ready to move")
                        break        

                # Set start sequence parameter to true
                rospy.loginfo("Starting {type} sequence".format(type=self.sequence_handler.current_sequence.type))
                rospy.set_param("start_sequence", True)

                while not rospy.is_shutdown():
                    # Check to see if motion was detected
                    if self.motion_detected:
                        pause_requested = True
                        rospy.loginfo("Motion was detected...pausing robots")
                        
                        # Cancel leader and follower goal
                        self.leader_follow_waypoints_client.cancel_goal()
                        self.follower_follow_frame_client.cancel_goal()
                        
                        # Get leader result and update sequence progress
                        self.leader_follow_waypoints_client.wait_for_result()
                        leader_result = self.leader_follow_waypoints_client.get_result()
                        
                        rospy.loginfo("The leader robot has paused")
                        if sequence.type == SequenceMsg.CIRCLE:
                            self.sequence_handler.update_circle_progress(leader_result.theta_c, leader_result.completed_rev)
                        elif sequence.type == SequenceMsg.TRIANGLE:
                            self.sequence_handler.update_triangle_progress(leader_result.side_name, leader_result.completed_rev)
                        break
                    
                    # Check that the leader is still okay
                    if self.leader_follow_waypoints_client.get_state() == actionlib.GoalStatus.ABORTED or self.leader_follow_waypoints_client.get_state() == actionlib.GoalStatus.LOST:
                        rospy.logerr("The leader was unable to reach the goal")
                        self.follower_follow_frame_client.cancel_goal()
                        return False

                    # Check that the follower is still okay
                    if self.follower_follow_frame_client.get_state() == actionlib.GoalStatus.ABORTED or self.follower_follow_frame_client.get_state() == actionlib.GoalStatus.LOST:
                        rospy.logerr("The follower was unable to reach the goal")
                        self.leader_follow_waypoints_client.cancel_goal()
                        return False
                    
                    # Check to see if leader successfully finished the sequence
                    if self.leader_follow_waypoints_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                        sequence_completed = True
                        rospy.loginfo("The leader has finished the sequence")
                        self.follower_follow_frame_client.cancel_goal()
                        break
                    
                # Wait for the follower to reach the final position
                while not rospy.is_shutdown():
                    follower_state = self.follower_follow_frame_client.get_state()
                    rospy.loginfo(follower_state)
                    if follower_state == actionlib.GoalStatus.ABORTED:
                        rospy.logerr("The follower was unable to reach the leader position before timeout")
                        return False

                    if follower_state == actionlib.GoalStatus.SUCCEEDED:
                        if sequence_completed:
                            rospy.loginfo("The follower has finished the sequence")
                        elif pause_requested:
                            rospy.loginfo("The follower robot has paused")
                        break
                
                if pause_requested:
                    first_attempt = False
                    # Wait until motion is cleared
                    while not rospy.is_shutdown():
                        if not self.motion_detected:
                            break
                        
            rospy.sleep(self.sequence_handler.get_delay())
            
        return True

    def bias_ft_sensor(self):
        set_bias = rospy.ServiceProxy('/bias', SetBias)

        try:
            request = SetBiasRequest()
            request.toBias = True
            request.forceMax = 100
            request.torqueMax = 10
            success = set_bias(request).success
            if success:
                rospy.loginfo("Set bias on f/t sensor")
            else:
                rospy.logwarn("Unable to bias f/t sensor")

        except rospy.ServiceException:
            rospy.logwarn("Unable to bias f/t sensor")

    def lookup_transform(self, child_frame, parent_frame):
        try:
            transform = self.tfBuffer.lookup_transform(parent_frame, child_frame, rospy.Time(), rospy.Duration(1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Unable to lookup transform between" + child_frame + " and " + parent_frame)
            return None

        return transform
    
    def leader_feedback_cb(self, feedback):
        self.leader_feedback = feedback
    
    def follower_feedback_cb(self, feedback):
        self.follower_feedback = feedback
        
    def motion_detected_cb(self, msg):
        self.motion_detected = msg.data
