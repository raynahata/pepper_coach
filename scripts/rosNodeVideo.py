#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from naoqi import ALProxy
import time

# Initialize Pepper Connection
IP = "128.237.236.27"  
PORT = 9559

# Camera settings
resolution = 2  # 640x480
color_space = 11  # RGB
fps = 5  # Frames per second

class PepperCameraPublisher:
    def __init__(self):
        rospy.init_node("pepper_camera_publisher", anonymous=True)
        self.video_service = ALProxy("ALVideoDevice", IP, PORT)
        self.publisher = rospy.Publisher("/pepper_camera/image_raw", Image, queue_size=10)
        self.subscriber_id = self.video_service.subscribeCamera("video_stream", 0, resolution, color_space, fps)

    def publish_frame(self):
        rate = rospy.Rate(fps)
        while not rospy.is_shutdown():
            image = self.video_service.getImageRemote(self.subscriber_id)
            if image:
                width = image[0]
                height = image[1]
                timestamp = rospy.Time.now()

                # Create ROS Image message
                ros_image = Image()
                #ros_image.header = Header()
                ros_image.header.stamp = timestamp
                ros_image.width = width
                ros_image.height = height
                ros_image.encoding = "rgb8"
                #ros_image.is_bigendian = 0
                #ros_image.step = width * 3  # 3 bytes per pixel (RGB)
                ros_image.data = image[6]  # Image pixel data

                # Publish to ROS topic
                self.publisher.publish(ros_image)
                rospy.loginfo("Published a frame from Pepper.")

            rate.sleep()
        camera_node.shutdown()

    def shutdown(self):
        rospy.loginfo("Shutting down Pepper camera node...")
        self.video_service.unsubscribe(self.subscriber_id)

if __name__ == "__main__":
    
    camera_node = PepperCameraPublisher()
    camera_node.publish_frame()
        
