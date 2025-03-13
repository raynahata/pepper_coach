from naoqi import ALProxy
import time

# Connect to Pepper
IP = "128.237.236.27"  # Replace with Pepper's IP address
PORT = 9559  # Default port

video_service = ALProxy("ALVideoDevice", IP, PORT)

# Subscribe to camera (0: Top, 1: Bottom, 2: Depth)
resolution = 2  # 640x480
color_space = 11  # RGB
fps = 10  # Frames per second

subscriber_id = video_service.subscribeCamera("video_stream", 0, resolution, color_space, fps)

# Capture a frame
image = video_service.getImageRemote(subscriber_id)

if image:
    width = image[0]
    height = image[1]
    array = image[6]  # Pixel array
    with open("frame.raw", "wb") as f:
        f.write(array)

# Unsubscribe to free resources
video_service.unsubscribe(subscriber_id)

print("Video frame captured!")
