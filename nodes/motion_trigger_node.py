#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse

class MotionTrigger:
    def __init__(self):
        self.motion = False
        self.period = None
        self.start_time = None
        
        try:
            rospy.wait_for_service('extend', timeout=5)
            rospy.wait_for_service('retract', timeout=5)
        except rospy.ROSException:
            rospy.logerr("Unable to connect to mechanism")
        
        # Service Clients
        self.extend = rospy.ServiceProxy('extend', Trigger)
        self.retract = rospy.ServiceProxy('retract', Trigger)
        
        # Service Servers
        self.start_motion_server = rospy.Service('start_motion', Trigger, self.start_motion)
        self.stop_motion_server = rospy.Service('stop_motion', Trigger, self.stop_motion)
        
    def start_motion(self, req):
        resp = TriggerResponse()
        
        try:
            rospy.wait_for_service('extend', timeout=1)
            rospy.wait_for_service('retract', timeout=1)
        except rospy.ROSException:
            rospy.logerr("Unable to connect to mechanism")
            resp.success = False
            return resp
        
        if self.motion:
            resp.success = False
            return resp
        
        self.period = rospy.get_param('motion_period')
        self.start_time = rospy.get_time()
        self.motion = True

        resp.success = True
        
        return resp
        
    def stop_motion(self, req):
        resp = TriggerResponse()
        
        try:
            rospy.wait_for_service('extend', timeout=1)
            rospy.wait_for_service('retract', timeout=1)
        except rospy.ROSException:
            rospy.logerr("Unable to connect to mechanism")
            resp.success = False
            return resp
        
        if not self.motion:
            resp.success = False
            return resp
        
        self.motion = False
        resp.success = True
        
        return resp
    
    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.motion:
                if rospy.get_time() - self.start_time >= self.period:
                    if not self.extend():
                        rospy.logerr("Unable to extend mechanism")
                    
                    rospy.sleep(0.5)
                    
                    if not self.retract():
                        rospy.logerr("Unable to retract mechanism")
                    
                    self.start_time = rospy.get_time()
            
            rate.sleep()
            

if __name__ == "__main__":
    rospy.init_node('motion_trigger_node')
    
    trig = MotionTrigger()
    
    trig.run()
