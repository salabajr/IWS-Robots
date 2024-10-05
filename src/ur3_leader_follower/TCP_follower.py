#!/usr/bin/env python

import math
import numpy as np
import geometry_msgs.msg
import rospy
import tf.transformations
import tf2_ros
import actionlib
import tf2_geometry_msgs
import PyKDL
import threading
import tf_conversions.posemath as pm
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerRequest
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from ur3_leader_follower.robot import Robot
from ur3_leader_follower.twist_controller import TwistController
from ur3_leader_follower.msg import Error
from ur3_leader_follower.msg import FollowFrameAction, FollowFrameFeedback
from std_msgs.msg import Float32


class Follower(Robot):
    def __init__(self):
        # Inherit everything from Robot class
        #test
        super(Follower, self).__init__("follower_")

        self.gripper_pin = 4
        self.home_joint_state = [-1.5708, -1.5708, -1.5708, 3.14159, -1.5708, 4.71239]
        self.base_link_to_world_transform = self.tfBuffer.lookup_transform("follower_base_link", "world", rospy.Time(), rospy.Duration(1))
        self.distance_error_threshold = 0.2 # 20 cm
        
        # Follow Frame Action Server
        self.follow_frame_server = actionlib.SimpleActionServer('follow_frame', FollowFrameAction,
                                                                self.follow_frame, auto_start=False)

        # Publishers
        self.joint_velocity_pub = rospy.Publisher("joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        
        # Feedback Publisher runs only if specified
        self.feedback = bool(rospy.get_param("feedback"))
        if  self.feedback == True:
            self.error_pub = rospy.Publisher("/leader/real_time_cartesian_error", Float32, queue_size=1)
            # Place publishing on seperate thread, publishes every 0.016 seconds currently
            self.t1 = threading.Thread(target = rospy.Timer(rospy.Duration.from_sec(0.016), self.update_offset_callback))
            self.t1.start()
            
        # Subscribers
        self.desired_pose = None
        self.desired_pose_time = None
        
        # Place receiving on a seperate thread
        self.t2 = threading.Thread(target=rospy.Subscriber("desired_tcp_pose", geometry_msgs.msg.PoseStamped, self.desired_pose_cb))
        self.t2.start()

        # IK Solver
        self.ik_vel_solver = PyKDL.ChainIkSolverVel_pinv(self.kdl_chain)

        # Twist Controller
        gains = {'l_p': float(rospy.get_param("kp")), 'l_i': float(rospy.get_param("ki")), 'l_d': float(rospy.get_param("kd")),
                 'a_p': 1, 'a_i': 0.1, 'a_d': 0}
        alpha = float(rospy.get_param("alpha"))  # filtering constant
        self.twist_controller = TwistController(gains, alpha)

    def update_offset_callback(self, event):
        cartesian_error = self.calculate_error()
        
        # Publish 0 error if the error can't be calculated
        if cartesian_error == None:
            self.error_pub.publish(0)
        else:
            self.error_pub.publish(cartesian_error.magnitude)

    def follow_frame(self, goal):
        self.twist_controller.reset()
        feedback = FollowFrameFeedback()
        
        # Check if velocity controller is running
        list_controllers = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        controllers = list_controllers(ListControllersRequest())
        
        running = False
        for state in controllers.controller:
            if state.name == "joint_group_vel_controller" and state.state == "running":
                running = True
                break
        
        if not running:
            if not self.switch_controller("joint_group_vel_controller"):
                self.follow_frame_server.set_aborted()
                return
            self.toggle_dashboard_program()

        feedback.ready_to_move = True
        self.follow_frame_server.publish_feedback(feedback)

        # Wait for sequence trigger from Supervisor
        while not rospy.get_param("/start_sequence"):
            pass
        
        # Start loop to send joint velocity commands
        preempt_request_time = None
        
        while not rospy.is_shutdown():
            # Calculate the error between desired pose and actual pose
    
            base_error = self.calculate_error()

            if base_error is None or base_error.magnitude > self.distance_error_threshold:
                if base_error is not None:
                    rospy.logerr("Error between action pose and desired pose exceeded threshold")
                
                self.stop()

                self.follow_frame_server.set_aborted()
                return False

            # Use the twist controller to generate the necessary twist at the end effector to move towards the target
            twist = self.twist_controller.calculate_twist(base_error)

            # Calculate the necessary joint velocities to achieve the calculated twist
            joint_velocities = self.calculate_joint_velocities(twist)

            # Publish the joint velocities
            #rospy.loginfo("Joint velocities")
            self.joint_velocity_pub.publish(joint_velocities)

            if self.follow_frame_server.is_preempt_requested():
                if not preempt_request_time:
                    preempt_request_time = rospy.get_rostime()

                if base_error.magnitude < 0.002 and max(np.abs(joint_velocities.data)) < 0.005:
                    self.stop()
                    self.follow_frame_server.set_succeeded()
                    return True

                elif rospy.get_rostime() > preempt_request_time + rospy.Duration(2):
                    rospy.logerr("Follower was unable to reach the target before the timeout")
                    self.stop()
                    self.follow_frame_server.set_aborted()                   
                    return False
                
        
    def calculate_error(self):
        global_error = Error()
        
        # Check if desired_pose exists
        if self.desired_pose is None:
            #rospy.logerr("The follower was unable to recieve data on desired pose topic")
            return None

        # Check if desired_pose data is fresh
        time_offset = abs(rospy.get_rostime() - self.desired_pose_time)
        if time_offset.to_sec() > 2:
            rospy.logerr("The desired pose data was older than threshold")
            return None

        actual_pose = self.get_tcp_pose()
        
        global_error.linear = geometry_msgs.msg.Vector3(actual_pose.position.x - self.desired_pose.position.x,
                                                        actual_pose.position.y - self.desired_pose.position.y,
                                                        actual_pose.position.z - self.desired_pose.position.z,)
        
        global_error.angular = self.calculate_angular_error(self.desired_pose.orientation, actual_pose.orientation)
        
        base_error = Error()
        
        base_error.linear = self.transform_vector(global_error.linear, self.base_link_to_world_transform)
        base_error.angular = self.transform_vector(global_error.angular, self.base_link_to_world_transform)
        
        base_error.magnitude = math.sqrt(base_error.linear.x ** 2 + base_error.linear.y ** 2 + base_error.linear.z ** 2)
        
        return base_error
    
    @staticmethod
    def calculate_angular_error(orientation1, orientation2):        
        q1_inv = [orientation1.x, orientation1.y, orientation1.z, -orientation1.w] 

        q2 = [orientation2.x, orientation2.y, orientation2.z, orientation2.w]

        qr = tf.transformations.quaternion_multiply(q2, q1_inv)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(qr)
        
        return geometry_msgs.msg.Vector3(roll, pitch, yaw)
    
    @staticmethod
    def transform_vector(vector, transform):
        tf_vector_stamped = tf2_geometry_msgs.Vector3Stamped()
        tf_vector_stamped.vector.x = vector.x
        tf_vector_stamped.vector.y = vector.y
        tf_vector_stamped.vector.z = vector.z
        transformed_vector = tf2_geometry_msgs.do_transform_vector3(tf_vector_stamped, transform)

        return transformed_vector.vector

    def calculate_joint_velocities(self, twist):
        joint_velocities = Float64MultiArray()
        joint_velocities.data = [0, 0, 0, 0, 0, 0]
        q_dot_out = PyKDL.JntArray(6)
        
        self.ik_vel_solver.CartToJnt(self.q_in, twist, q_dot_out)
        
        for idx, q in enumerate(q_dot_out):
            joint_velocities.data[idx] = q
        
        return joint_velocities
    
    def stop(self):
        joint_velocities = Float64MultiArray()
        joint_velocities.data = [0, 0, 0, 0, 0, 0]
        
        # Publish 0 velocity on all joints for 200ms
        start = rospy.get_rostime()
        while rospy.get_rostime() < start + rospy.Duration.from_sec(0.2):
            self.joint_velocity_pub.publish(joint_velocities)
            
    def desired_pose_cb(self, msg):
        #rospy.loginfo("Received")
        self.desired_pose_time = rospy.get_rostime()
        self.desired_pose = msg.pose
