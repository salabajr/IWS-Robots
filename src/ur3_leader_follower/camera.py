# !/usr/bin/env python

import rospy
import cv2
import time
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from ur3_leader_follower.srv import StartRecording, StartRecordingResponse
from cv_bridge import CvBridge, CvBridgeError

class Camera():
    camera_hostname = '10.10.40.1'
    fps = 0 
    frame_width = 1280
    frame_height = 720

    def __init__(self):
        self.url = "rtsp://{hostname}/axis-media/media.amp?fps={fps}&resolution={w}x{h}".format(hostname=self.camera_hostname, fps=self.fps, w=self.frame_width, h=self.frame_height)
        self.video_capture = cv2.VideoCapture(self.url)
        rospy.loginfo("Connected to camera")

        self.recording_status = "not started"
        self.static_back = None
        self.motion_detected = False
        self.clear_time = 3
        self.motion_publisher = rospy.Publisher('motion_detected', Bool, queue_size=10)
        self.motion_start_time = None

        # Create cv bridge
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("motion_detection_image", Image, queue_size=5)
        
        self.start_recording_srv = rospy.Service('start_recording', StartRecording, self.start_recording)
        self.stop_recording_srv = rospy.Service('stop_recording', Trigger, self.stop_recording)
    
    def start_recording(self, req):
        response = StartRecordingResponse()
        
        if self.recording_status == "started":
            response.success = False
            response.message = "Unable to start recording because recording was already started"
            return response

        self.filename = "/home/sensor/recordings/" + req.testname + "_video.mp4"
        self.video_writer = cv2.VideoWriter(self.filename, cv2.VideoWriter_fourcc(*'mp4v'), 60, 
                                            (int(self.video_capture.get(3)), int(self.video_capture.get(4))))
    
        rospy.loginfo("Recording Started")
        self.recording_status = "started"

        response.success = True
        return response

    def stop_recording(self, req):
        response = TriggerResponse()
        
        if not self.recording_status == "started":
            response.success = False
            response.message = "Unable to stop recording because recording was not yet started"
            return response
        else:
            self.recording_status = "stopped"
            rospy.loginfo("Recording Stopped")
            response.success = True
            return response
        
    def read_frames(self):
        while self.video_capture.isOpened():
            ret, frame = self.video_capture.read()
            if self.recording_status == "started":
                self.video_writer.write(frame)
            elif self.recording_status == "stopped":
                self.video_writer.release()
            
            if rospy.get_param("detect_motion", default=False):
                self.motion_detected = self.detect_motion(frame)

                if self.motion_detected:
                    self.motion_start_time = rospy.get_rostime()
                    msg = Bool()
                    msg.data = True
                    self.motion_publisher.publish(msg)
                
                if not self.motion_detected and self.motion_start_time is not None:
                    duration = rospy.get_rostime() - self.motion_start_time
                    if duration.to_sec() >= 3:
                        msg = Bool()
                        msg.data = False
                        self.motion_publisher.publish(msg)
                        self.motion_start_time = None
    
    def detect_motion(self, frame):
        # Crop to bottom of frame
        crop = frame[int(2*frame.shape[0]/3):frame.shape[0] - 50, 150:frame.shape[1]-150]
    
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
    
        # Set the static background on first frame
        if self.static_back is None:
            self.static_back = gray
            return False
    
        diff_frame = cv2.absdiff(self.static_back, gray)
    
        thresh_frame = cv2.threshold(diff_frame, 20, 255, cv2.THRESH_BINARY)[1]
        thresh_frame = cv2.dilate(thresh_frame, None, iterations = 2)
    
        _, cnts, _ = cv2.findContours(thresh_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
        motion = False
        for contour in cnts:
            if cv2.contourArea(contour) < 1000:
                continue
            (x, y, w, h) = cv2.boundingRect(contour)
            # making green rectangle around the moving object
            cv2.rectangle(crop, (x, y), (x + w, y + h), (0, 255, 0), 3)
            motion = True
        
        self.publish_image(crop)

        return motion
        
    def publish_image(self, frame):
        image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(image)

    def shutdown(self):
        rospy.loginfo("Shutting Down")
        self.video_capture.release()


