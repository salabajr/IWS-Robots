# !/usr/bin/env python

import rospy
import sys
import math
import actionlib
import kdl_parser_py.urdf
import PyKDL
import tf2_ros
import tf_conversions.posemath as pm
from std_srvs.srv import Trigger, TriggerRequest ,TriggerResponse
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from sensor_msgs.msg import JointState
import moveit_msgs.msg
import tf2_geometry_msgs
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal, FollowCartesianTrajectoryResult, CartesianTrajectory, CartesianTrajectoryPoint 
from ur_msgs.srv import SetIO, SetIORequest
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from ur3_leader_follower.srv import ActuateGripper, ActuateGripperResponse, GoToPose, GoToPoseResponse

class Robot(object):
    def __init__(self, prefix=""):
        self.prefix = prefix
        self.home_joint_state = None
        self.joint_names = None

        # KDL
        ok, kdl_tree = kdl_parser_py.urdf.treeFromParam("robot_description")
        if not ok:
            rospy.logerr("Unable to create kdl tree from robot description")
        self.kdl_chain = kdl_tree.getChain(self.prefix + "base_link", self.prefix + "tcp")
        self.fk_pos_solver = PyKDL.ChainFkSolverPos_recursive(self.kdl_chain)
        self.q_in = PyKDL.JntArray(self.kdl_chain.getNrOfJoints())
        self.joint_state_sub = rospy.Subscriber("joint_states", JointState, self.joint_state_callback)

        # TF
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        
        # Determine KDL Frame from world coordinates to base_link
        wtb = self.tfBuffer.lookup_transform("world", self.prefix + "base_link", rospy.Time(), rospy.Duration(1)).transform
        self.world_to_base_frame = PyKDL.Frame(PyKDL.Rotation.Quaternion(wtb.rotation.x, wtb.rotation.y, wtb.rotation.z, wtb.rotation.w),
                                               PyKDL.Vector(wtb.translation.x, wtb.translation.y, wtb.translation.z))

        # Service Servers
        self.actuate_gripper_server = rospy.Service('actuate_gripper', ActuateGripper, self.actuate_gripper)
        self.gripper_pin = None  # Digital I/O pin for gripper
        self.go_home_server = rospy.Service('go_home', Trigger, self.go_home)
        self.go_to_pose_server = rospy.Service('go_to_pose', GoToPose, self.go_to_pose)

        # Service Clients
        self.set_io = rospy.ServiceProxy("ur_hardware_interface/set_io", SetIO) 
        self.controller_switcher = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)
        self.dashboard_play = rospy.ServiceProxy('ur_hardware_interface/dashboard/play', Trigger)
        self.dashboard_pause = rospy.ServiceProxy('ur_hardware_interface/dashboard/pause', Trigger)

        # Controller Action Clients
        self.joint_trajectory_client = actionlib.SimpleActionClient("scaled_pos_joint_traj_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.cart_trajectory_client = actionlib.SimpleActionClient("pose_based_cartesian_traj_controller/follow_cartesian_trajectory", FollowCartesianTrajectoryAction)
        self.ee_speed = 0.1  # m/s 
        
        # Controller Lists
        self.joint_trajectory_controllers = ["scaled_pos_joint_traj_controller", "scaled_vel_joint_traj_controller", "pos_joint_traj_controller", "vel_joint_traj_controller", "forward_joint_traj_controller",]
        self.cartesian_trajectory_controllers = ["pose_based_cartesian_traj_controller", "joint_based_cartesian_traj_controller", "forward_cartesian_traj_controller"]
        self.conflicting_controllers = ["joint_group_vel_controller", "twist_controller"]

    def actuate_gripper(self, req):
        robot_request = SetIORequest()
        robot_request.state = req.gripper_state
        robot_request.pin = self.gripper_pin                
        robot_request.fun = 1
        result = self.set_io(robot_request)

        response = ActuateGripperResponse()
        response.success = result.success

        return response

    def go_home(self, req):
        response = TriggerResponse()

        # Check if already in the home joint state
        error = 0
        for i, pos in enumerate(self.home_joint_state):
            error += abs(pos - self.q_in[i])
        if error < 0.01:
            response.success = True
            return response
        
        # Switch to the correct controller
        if not self.switch_controller("scaled_pos_joint_traj_controller"):
            response.success = False
            return response

        self.joint_trajectory_client.wait_for_server()
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        
        # Set the joint positions to the home joint state
        point = JointTrajectoryPoint()
        point.positions = self.home_joint_state
        
        # Compute the distance from the current pose to the home pose to determine how fast to move
        home_joint_array = PyKDL.JntArray(len(self.home_joint_state))
        for i, pos in enumerate(self.home_joint_state):
            home_joint_array[i] = pos
            
        out_frame = PyKDL.Frame()
        
        self.fk_pos_solver.JntToCart(home_joint_array, out_frame)
        home_pose = pm.toMsg(self.world_to_base_frame * out_frame)

        travel_time = self.distance(self.get_tcp_pose(), home_pose)/0.1
        if travel_time < 0.5:
            travel_time = 0.5
        point.time_from_start = rospy.Duration.from_sec(travel_time)

        goal.trajectory.points = [point]
        
        # Send goal and wait for result
        self.joint_trajectory_client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(5))
        result = self.joint_trajectory_client.get_result()
        
        # Send back response based on result
        if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
            response.success = True
            
        else:
            rospy.logerr(result.error_code)
            response.success = False
        
        return response
        
    def go_to_pose(self, req):
        response = GoToPoseResponse()
        
        # Check if correct controller is running
        list_controllers = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        controllers = list_controllers(ListControllersRequest())
        
        running = False
        for state in controllers.controller:
            if state.name == "pose_based_cartesian_traj_controller" and state.state == "running":
                running = True
                break
        
        # If not switch controller and toggle dashboard
        if not running:
            if not self.switch_controller("pose_based_cartesian_traj_controller"):
                response.success = False
                return response
            self.toggle_dashboard_program()
        
        # Establish goal
        goal = FollowCartesianTrajectoryGoal()
        goal.trajectory = self.generate_cartesian_trajectory([req.goal_pose])
        goal.trajectory.points[0].time_from_start = req.time_from_start
        
        # Wait for server
        success = self.cart_trajectory_client.wait_for_server(timeout=rospy.Duration(1))
        if not success:
            rospy.logerr("Unable to connect to the trajectory client")
            response.success = False
            return response
        
        num_tries = 0
        while num_tries < 5:
            # Send goal and wait for result
            self.cart_trajectory_client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(5))
            result = self.cart_trajectory_client.get_result()

            distance = self.distance(req.goal_pose.pose, self.get_tcp_pose())

            if result.error_code != FollowCartesianTrajectoryResult.SUCCESSFUL:
                rospy.logerr(result.error_code)
                response.success = False
                return response
            
            if distance < 0.005:
                response.success = True
                return response
            
            rospy.loginfo("Robot unable to go to requested pose...trying again")
            num_tries += 1

        rospy.logerr("Robot failed to reach pose after five tries. Exiting")
        response.success = False
        return response
        
    def joint_state_callback(self, msg):
        self.joint_names = msg.name
        self.q_in[0] = msg.position[2]
        self.q_in[1] = msg.position[1]
        self.q_in[2] = msg.position[0]
        self.q_in[3] = msg.position[3]
        self.q_in[4] = msg.position[4]
        self.q_in[5] = msg.position[5]

    def get_tcp_pose(self, r_type='msg'):
        # Use KDL FK solver to find tcp pose in base_link frame
        base_to_tcp_frame = PyKDL.Frame()
        self.fk_pos_solver.JntToCart(self.q_in, base_to_tcp_frame)

        # Find tcp pose in world frame
        world_to_tcp_frame = self.world_to_base_frame * base_to_tcp_frame

        if r_type == 'msg':
            return pm.toMsg(world_to_tcp_frame)
        elif r_type == 'frame':
            return world_to_tcp_frame
        
    def generate_cartesian_trajectory(self, waypoints, stop_after_each=False):
        trajectory = CartesianTrajectory()
        
        # Calculate initial time from start
        start_pose = self.get_tcp_pose()
        time_from_start = self.distance(start_pose, waypoints[0].pose)/self.ee_speed
    
        # Create a cartesian trajectory point for each point in waypoints
        for i, stamped_pose in enumerate(waypoints):
            point = CartesianTrajectoryPoint()
            
            # transform pose into base frame
            target_frame = self.prefix + 'base'
            
            transform = self.tfBuffer.lookup_transform(target_frame, stamped_pose.header.frame_id, rospy.Time(), rospy.Duration(1))
            
            point.pose = tf2_geometry_msgs.do_transform_pose(stamped_pose, transform).pose
            point.time_from_start = rospy.Duration.from_sec(time_from_start)
            
            if not stop_after_each:
                point.twist.linear.x = float('NaN')
            
            trajectory.points.append(point)
            
            if not i == len(waypoints) - 1:
                time_from_start += self.distance(stamped_pose.pose, waypoints[i+1].pose)/self.ee_speed
        
        return trajectory
    
    def switch_controller(self, target_controller):
        stop_controllers = (self.joint_trajectory_controllers + self.cartesian_trajectory_controllers + self.conflicting_controllers)
        
        if target_controller in stop_controllers:
            stop_controllers.remove(target_controller)
        else:
            rospy.logerr("{controller} is not a valid controller name".format(controller=target_controller))
            return False
        
        req = SwitchControllerRequest()
        req.stop_controllers = stop_controllers
        req.start_controllers = [target_controller]
        req.strictness = SwitchControllerRequest.BEST_EFFORT
        
        try:
            self.controller_switcher.wait_for_service(timeout=rospy.Duration(1))
        except rospy.ROSException:
            rospy.logerr("Unable to connect to controller switcher")
            return False

        try:
            response = self.controller_switcher(req)
        except rospy.ServiceException:
            rospy.logerr("Unable to connect to controller switcher")
            return False
        
        if not response.ok:
            rospy.logerr("Unable to switch to {controller}".format(controller=target_controller))
            return False
        
        return True
    
    def toggle_dashboard_program(self):
        pause_client = rospy.ServiceProxy('ur_hardware_interface/dashboard/pause', Trigger)
        play_client = rospy.ServiceProxy('ur_hardware_interface/dashboard/play', Trigger)
        
        result = pause_client(TriggerRequest())

        if not result.success:
            rospy.loginfo("Unable to pause dashboard program")
            return False

        rospy.sleep(0.5)
        
        result = play_client(TriggerRequest())
        
        if not result.success:
            rospy.loginfo("Unable to play dashboard program")
            return False

        return True
            
    @staticmethod
    def distance(pt1, pt2):
        '''Calculates the linear distance between two geometry_msgs/Pose in the same frame'''
        return math.sqrt((pt1.position.x - pt2.position.x)**2 +
                         (pt1.position.y - pt2.position.y)**2 +
                         (pt1.position.z - pt2.position.z)**2)
