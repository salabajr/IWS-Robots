#!/usr/bin/env python

import rospy
from ur3_leader_follower.supervisor import Supervisor

if __name__ == "__main__":
    rospy.init_node('supervisor_node')
    supervisor = Supervisor()

    if not supervisor.sequence_handler.read_sequences():
        exit()

    # Wait for leader and follower action servers to be advertised
    supervisor.leader_follow_waypoints_client.wait_for_server()
    supervisor.follower_follow_frame_client.wait_for_server()

    supervisor.send_home('leader')
    supervisor.send_home('follower')
    
    if not supervisor.send_robots_to_grip_bar():
        rospy.logerr("Robots were unable to grasp bar")
        exit()

    response = str(raw_input("Start sequences (y/n)?: "))

    if response.lower() == "y":
        # Start camera recording
        started_recording = False
        if rospy.get_param("record_camera"):
            testname = rospy.get_param('test_name')
            started_recording = supervisor.start_camera_recording(testname)
            
        # Set the bias on the f/t sensor
        supervisor.bias_ft_sensor()
        
        # Start motion
        if rospy.get_param("enable_motion"):
            supervisor.start_motion()
        
        # Execute the sequences
        success = supervisor.execute_sequences()
    
        # Stop camera recording
        if started_recording:
            supervisor.stop_camera_recording()
            
        # Stop motion
        if rospy.get_param("enable_motion"):
            supervisor.stop_motion()

        if not success:
            rospy.logerr("Unable to execute all sequences")
            exit()

    if not supervisor.return_bar_to_stand():
        rospy.logerr("Robots were unable to return bar to stand")
        exit()
    
    supervisor.send_home('leader')
    supervisor.send_home('follower')
