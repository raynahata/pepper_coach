# Video recorder final code
import os
import sys
import time
from naoqi import ALProxy
#import cv2

# Replace this with your robot's IP address
IP = "128.237.236.27"
PORT = 9559

save_folder = "/home/nao/recordings/cameras"

# Create a proxy to ALVideoRecorder
videoRecorderProxy = ALProxy("ALVideoRecorder", IP, PORT)


videoRecorderProxy.setFrameRate(10.0)
videoRecorderProxy.setResolution(2) # Set resolution to VGA (640 x 480)
# We'll save a 5 second video record in /home/nao/recordings/cameras/
videoRecorderProxy.startRecording("/home/nao/recordings/cameras", "test")
print("Video record started.")

time.sleep(5)

videoInfo = videoRecorderProxy.stopRecording()
print ("Video was saved on the robot: ", videoInfo[1])
print ("Total number of frames: ", videoInfo[0])

'''# Save or display the image (optional)
frame = videoRecorderProxy.getFrame()
if frame is not None:
    curr_image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    frame_filename = os.path.join(save_folder, "frame.png")
    cv2.imwrite(frame_filename, curr_image)'''

#scp nao@128.237.236.27:/home/nao/recordings/cameras/test.avi Downloads/