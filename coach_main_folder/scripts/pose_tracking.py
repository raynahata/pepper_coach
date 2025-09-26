#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from pytz import timezone
from datetime import datetime
import numpy as np
from sensor_msgs.msg import Image
# from config import *
import cv2
from cv_bridge import CvBridge
import yaml

from rospkg import RosPack

import warnings
warnings.filterwarnings("ignore")
# from config import *

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import matplotlib.pyplot as plt 

rp = RosPack()
bridge = CvBridge()

class PoseTracking:

    def __init__(self): 
        #initialize subscriber - listens, and every time image published, run callback function
        #arg 1: camera is publishing messages to a topic (/astra_ros/devices/default/color/image_color)
        #arg 2: type of message is Image
        #arg 3: callback function - every time image input is received, callback function is called (gets angles).
        #self.sub = rospy.Subscriber("/astra_ros/devices/default/color/image_color", Image, self.callback)
        
        #TODO: check rostopics and see pepper camera
        self.sub = rospy.Subscriber("/pepper_camera/image_raw", Image, self.callback)
        self.image_pub = rospy.Publisher("/annotated_image", Image, queue_size=10)
        self.computer_params = self.load_params()

        #have a library that sets up pose estimator
        #use points on body as landmark points
        self.landmark_points = ['nose', 'left_eye_inner', 'left_eye', 'left_eye_outer', 'right_eye_inner', 'right_eye', 'right_eye_outer', 'left_ear', 'right_ear', 'mouth_left', 'mouth_right', 'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow', 'left_wrist', 'right_wrist', 'left_pinky', 'right_pinky', 'left_index', 'right_index', 'left_thumb', 'right_thumb', 'left_hip', 'right_hip', 'left_knee', 'right_knee', 'left_ankle', 'right_ankle', 'left_heel', 'right_heel', 'left_foot_index', 'right_foot_index']
        #pose_landmarker.task is a model file, trained externally on skeletons

        #TODO: modify path of file
        base_options = python.BaseOptions(model_asset_path=rp.get_path('pepper_exercise_coach') + "/config/pose_landmarker.task")
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=False)
        #Creating pose detector
        self.pose_detector = vision.PoseLandmarker.create_from_options(options)
        
        # self.pose_detector = mp.solutions.pose.Pose(
        #             min_detection_confidence=0.5,  # have some confidence baseline
        #             min_tracking_confidence=0.5,
        #             model_complexity=0,)
        self.flag = False
    
    def load_params(self):
        with open(rp.get_path('pepper_exercise_coach')+"/config/computer_config.yaml", "r") as file:
            data = yaml.safe_load(file)
        return data

    def calc_angle(self, vec_0, vec_1, angle_type):
        if angle_type == 'xy':
            angle = np.arctan2(vec_1[1], vec_1[1]) - \
                np.arctan2(-vec_0[1], vec_0[0])
        elif angle_type == 'yz':
            angle = np.arctan2(vec_1[1], vec_1[2]) - \
                np.arctan2(-vec_0[1], -vec_0[2])
        elif angle_type == 'xz':
            angle = np.arctan2(vec_1[2], vec_1[0]) - \
                np.arctan2(-vec_0[2], -vec_0[0])
        
        angle = np.abs(angle*180.0/np.pi)
        if angle > 180:
            angle = 360-angle

        return 180 - angle


    #callback function
    def callback(self, data):
        if not self.flag:
            return
        #takes in image as data
        #reformatting image so pose detector can understand (getting image in mumpy format)
        image = np.frombuffer(data.data, dtype=np.uint8).reshape(
            data.height, data.width, -1)
        annotated_image = image.copy()
        #create mp image from numpy input directly since it's already in rgb format
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        
        #visualize input image using cv2
        #convert from rgb to bgr
        # image = image[...,::-1]
        # cv2.imwrite('test.jpg', image) 
        # image = mp.Image.create_from_file('test.jpg')

        #running image through pose detector
        #detect function - gets 3d position of each of the landmarks
        results = self.pose_detector.detect(mp_image)
        #results are the 3d positions of the landmarks
        if results.pose_landmarks:
            ct = datetime.now(tz)

            landmarks = []
            image_landmarks = []

            #iterating through names of all the landmarks, and storying xyz coordinates
            for i, landmark in enumerate(results.pose_landmarks[0]):
                #creates a list of landmarks, and stores list of coordinates for each landmark
                landmarks.append([landmark.x, landmark.y, landmark.z])
                #adjust original iamge 
                if landmark.x <= 1.0 or landmark.y <= 1.0:
                    image_landmarks.append([int(landmark.x * image.shape[1]), int(landmark.y * image.shape[0])])
            
            for point in image_landmarks:
                cv2.circle(annotated_image, point, 5, (255, 0, 0), -1)  # Red circles around landmarks

            ros_image = bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            self.image_pub.publish(ros_image)

            angle_msg = Float64MultiArray()
            data = []

            # calculating angles - see src/quori_exercises/exercise_session/config_computer.py
            # for each joint_group (refers to the larger category), angle_description (the three other landmarks it refers to)
            # this loop repeats 4 times, because 4 joints looking through in ANGLE_INFO
            for joint_group, angle_description in self.computer_params['angle_info'].items():
                #in all landmarks found, care about 3 things (ex; right hip, right shoulder, right elbow)
                #for each one, find the index in the global list of landmarks (will have 3 indexes)
                indices = [self.landmark_points.index(angle_description[i]) for i in range(3)]
                
                #xyz coordinates for each of the three indexes
                #numpy array of xyz, xyz, xyz 
                points = [np.array(landmarks[i]) for i in indices]

                #now have coordinates, but need angles
                #turn into vectors
                #1 is middle point
                vec_0 = points[0] - points[1]
                vec_1 = points[2] - points[1]

                for plane in ['xy', 'yz', 'xz']:
                    angle = self.calc_angle(vec_0, vec_1, plane)
                    data.append(angle)
            
            angle_msg.data = data
            # print(len(data))
            angle_pub.publish(angle_msg)


if __name__ == '__main__':
    
    rospy.init_node('pose_tracking', anonymous=True)
    
    angle_pub = rospy.Publisher('joint_angles', Float64MultiArray, queue_size=10)
    # face_pub = rospy.Publisher('facial_features', Float64MultiArray, queue_size=10) unused
    tz = timezone('EST')

    #Start with exercise 1, set 1
    pose_tracking = PoseTracking()
    inittime = datetime.now(tz)

    pose_tracking.flag = True
    rospy.spin()