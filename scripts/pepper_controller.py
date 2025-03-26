import sys
from naoqi import ALProxy
# import matplotlib.pyplot as++ plt+

import rospy
import yaml
import math
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rospkg import RosPack

# Replace this with your Pepper's IP address
pepper_ip = "128.237.236.27"
pepper_port = 9559
action_received = False

rp = RosPack()


class Pepper:
    def __init__(self):
        self.IP = "128.237.236.27"
        self.tts = ALProxy("ALTextToSpeech", self.IP, 9559)
        self.motion = ALProxy("ALMotion", self.IP, 9559)
        self.posture = ALProxy("ALRobotPosture", self.IP, 9559)
        self.life = ALProxy('ALAutonomousLife', self.IP, 9559)
        self.life.setAutonomousAbilityEnabled("All", False)
        self.life.stopAll()
        self.tablet = ALProxy("ALTabletService", self.IP, 9559)

        self.tts.setParameter("defaultVoiceSpeed", 70)
        self.tts.setParameter("pitchShift", 1)
        self.exercise_running = False

        self.pepper_params = self.load_params()
        
        self.is_action_received = False
        self.is_executing_action = False

        #initialize camera
        resolution = 2  # 640x480
        color_space = 11  # RGB
        fps = 5  # Frames per second
        self.video_service = ALProxy("ALVideoDevice", self.IP, 9559)
        self.subscriber_id = self.video_service.subscribeCamera("video_stream", 0, resolution, color_space, fps)
        
        self.state = ""
        self.current_text = ""
        self.action_flag = ""

        # ROS Publishers and Subscribers
        self.angle_publisher = rospy.Publisher("arm_angles", String, queue_size=10)
        self.state_pub = rospy.Publisher("pepper_state", String, queue_size=10)
        self.text_pub = rospy.Publisher("chat_text", String, queue_size=10)
        self.exercise_publisher = rospy.Publisher("/exercise_command", String, queue_size=10)
        self.image_publisher = rospy.Publisher("/pepper_camera/image_raw", Image, queue_size=10)

        rospy.Subscriber("pepper_state", String, self.callback_state)


        rospy.loginfo("Subscribed to /gpt_speech")

        self.exercise_running = False  # True when an exercise is running
        self.current_exercise = None  # Stores the name of the current exercise
        self.is_resting = False  # True when Pepper is resting

        rospy.loginfo("Subscribed to /exercise_command topic.")

    ### Helper Function: Convert Degrees to Radians ###
    def degrees_to_radians(self, angles_in_degrees):
        """
        Convert a list of angles from degrees to radians.
        Args:
            angles_in_degrees: List of angles in degrees.
        Returns:
            List of angles in radians.
        """
        return [angle * math.pi / 180.0 for angle in angles_in_degrees]

    ### Function to Move Arms ###
    def move_arm(self, side, angles, speed=0.2):
        """
        Move Pepper's arm to the specified angles.
        Args:
            side: "R" for right arm, "L" for left arm.
            angles: List of angles (in radians) for the arm's joints.
            speed: Fraction of maximum speed (0.0 to 1.0).
        """
        #publish the angles that are fed into this method

        ##TODO: listen to angles from pepper robot
        # parsed_angles = ' '.join("" + angle for angle in angles)
        # self.angle_publisher.publish(parsed_angles)

        if side == "R":
            joint_names = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
        elif side == "L":
            joint_names = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
        else:
            rospy.logerr("Invalid side specified. Use 'R' for right arm or 'L' for left arm.")
            return

        if len(angles) != len(joint_names):
            rospy.logerr("Number of angles does not match the number of joints.")
            return

        # rospy.loginfo("Moving {} arm to angles: {}".format(side, angles))
        self.motion.setAngles(joint_names, angles, speed)
        # self.motion.angleInterpolation(joint_names,angles,[speed]*len(joint_names),True)

    ### Function to Move Neck ###
    def move_neck(self, angles, speed=0.2):
        """
        Moves Pepper's torso to the specified angles
        Args:
            angles: List of angles (in radians) for the neck joints
            speed: Fraction of the maximum speed (0.0 to 1.0)
        """
        joint_names = ["HeadYaw", "HeadPitch"]
        if len(angles) != len(joint_names):
            rospy.logerr("Number of angles does not match the number of neck joints.")
            return
        self.motion.setAngles(joint_names, angles, speed)

    ### Function to Move Torso ###
    def move_torso(self, angles, speed=0.2):
        """
        Moves Pepper's torso to the specified angles
        Args:
            angles: List of angles (in radians) for the torso joints
            speed: Fraction of the maximum speed (0.0 to 1.0)
        """
        joint_names = ["HipRoll", "HipPitch", "KneePitch"]
        if len(angles) != len(joint_names):
            rospy.logerr("Number of angles does not match the number of torso joints.")
            return

        # rospy.loginfo("Moving torso to angles: {}".format(angles))
        self.motion.setAngles(joint_names, angles, speed)

    def say_text(self, text):
        """
        Make Pepper say the provided text.
        """
        rospy.loginfo("Saying: {}".format(text))
        self.tts.say(text)

    def set_flag_listening(self):
        """
        Set state to 'listening' and trigger appropriate action.
        """
        self.state = "listening"
        self.state_pub.publish("listening")

    def set_flag_speaking(self):
        """
        Set state to 'speaking' and trigger appropriate action.
        """
        self.state = "speaking"
        self.state_pub.publish("speaking")

    def callback_state(self, data):
        """
        Callback for 'pepper_state' topic.
        """
        rospy.loginfo("Received state: {}".format(data.data))
        self.state = data.data

    def publish_text(self, text):
        """
        Publish text to the 'chat_text' topic.
        """
        rospy.loginfo("Publishing text: {}".format(text))
        self.text_pub.publish(text)

    def clear_screen(self):
        rospy.loginfo("Clearing Pepper's tablet screen.")
        js_script = """document.body.innerHTML = `<style>body{background:#f0f0f0;margin:0;}</style>`;"""
        self.tablet.executeJS(js_script)

    def display_text(self, message):
        """
        Displays animated scrolling text on Pepper's tablet using JavaScript.
        """

        rospy.loginfo("Displaying static text on tablet: {}".format(message))
        js_script = """document.body.innerHTML = `<style>body{font-family:Arial,sans-serif;text-align:center;background:#f0f0f0;display:flex;justify-content:center;align-items:center;height:100vh;width:100vw;margin:0;padding:20px;overflow:hidden;} .text{font-size:10vh;color:#333;width:90vw;height:100vh;word-wrap:break-word;overflow-wrap:break-word;display:flex;align-items:center;justify-content:center;text-align:center;white-space:normal;line-height:1.5;}</style><div class='text'>""" + message + """</div>`;"""
        self.tablet.executeJS(js_script)

    ### Hardcoded Arm Motion: Up ###
    def stop_exercise_motion(self):
        self.motion.stopMove()
        self.motion.setStiffnesses("Body", 0.0)

    ### Begin code for Pepper's movements in response to rating/movement ###
    def move_arm_callback(self, msg):
        rospy.loginfo("Received move arm command: {}".format(msg.data))
        command = msg.data.lower()
        if command != self.action_flag:
            self.action_flag = command
            self.is_action_received = True
        else:
            rospy.loginfo("Ignoring duplicate move arm command: {}".format(msg.data))

    def listener(self):
        """
        Start the ROS listener node and execute the arm motion loop.
        """
        rospy.loginfo("Starting listener...")
        self.move_arms_up_and_down()
    
    def pub_image(self):
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
            ros_image.step = width * 3  # 3 bytes per pixel (RGB)
            ros_image.data = image[6]  # Image pixel data

            # Publish to ROS topic
            self.image_publisher.publish(ros_image)
            # rospy.loginfo("Published a frame from Pepper.")

    def execute_parsed_movements(self, part_name, radian_positions):
        """
        Helper function for parse_all_movements, purpose is to parse which part the movement
        refers to and move the correct part with the according input radian positions
        """
        if part_name == 'right_arm_degrees':
            self.move_arm("R", radian_positions, speed=0.1)
        elif part_name == 'left_arm_degrees':
            self.move_arm("L", radian_positions, speed=0.1)
        elif part_name == 'torso_degrees':
            self.move_torso(radian_positions, speed=0.1)
        elif part_name == 'neck_degrees':
            self.move_neck(radian_positions, speed=0.1)

    def parse_all_movements(self, all_positions):
        """
        Helper function for execute_pepper_action, purpose is to parse the part name and positions of
        the part and pass it in to execute_parsed_movements for actual executions
        """
        for position in all_positions:
            part_name = position[0]
            degree_positions = position[1]
            radian_positions = self.degrees_to_radians(degree_positions)
            self.execute_parsed_movements(part_name, radian_positions)
    
    def execute_pepper_action(self):
        if self.action_flag == "firm":
            rospy.loginfo("Going to firm position")
            #self.firm_position_action()
            all_positions = self.pepper_params['feedback_position_info']['firm_positions']
            self.parse_all_movements(all_positions)
            self.is_executing_action = True
        elif self.action_flag == "encouraging":
            rospy.loginfo("Going to encouraging position")
            #self.encouraging_position_action()
            all_positions = self.pepper_params['feedback_position_info']['encouraging_positions']
            self.parse_all_movements(all_positions)
            self.is_executing_action = True
        elif self.action_flag == "neutral":
            rospy.loginfo("Going to neutral position")
            # self.neutral_position_action()
            all_positions = self.pepper_params['feedback_position_info']['neutral_positions']
            self.parse_all_movements(all_positions)
        else:
            rospy.logwarn("Action of " + str(self.action_flag) + " not implemented!")
    
    def shutdown_camera(self):
        rospy.loginfo("Shutting down Pepper camera node...")
        self.video_service.unsubscribe(self.subscriber_id)

    def check_action_execution(self):
        #clarify corresponding parts moved with param indices
        r_arm_idx = 0
        l_arm_idx = 1
        degrees_idx = 1
        #define joint names
        r_joint_names = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
        l_joint_names = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
        #grab joint angles
        r_joint_angles = self.motion.getAngles(r_joint_names, True)
        l_joint_angles = self.motion.getAngles(l_joint_names, True)
        #check joint angles for different actions
        if self.action_flag == "firm":
            #grab left and right arm commands
            r_arm_degrees = self.pepper_params['feedback_position_info']['firm_positions'][r_arm_idx][degrees_idx]
            r_arm_radians = self.degrees_to_radians(r_arm_degrees)
            l_arm_degrees = self.pepper_params['feedback_position_info']['firm_positions'][l_arm_idx][degrees_idx]
            l_arm_radians = self.degrees_to_radians(l_arm_degrees)
        elif self.action_flag == "encouraging":
            r_arm_degrees = self.pepper_params['feedback_position_info']['encouraging_positions'][r_arm_idx][degrees_idx]
            r_arm_radians = self.degrees_to_radians(r_arm_degrees)
            l_arm_degrees = self.pepper_params['feedback_position_info']['encouraging_positions'][l_arm_idx][degrees_idx]
            l_arm_radians = self.degrees_to_radians(l_arm_degrees)
        #iterate through each and determine if current value close to actual
        for curr_joint_angle, cmd_joint_angle in zip(r_joint_angles, r_arm_radians):
            if abs(curr_joint_angle - cmd_joint_angle) > 0.1:
                self.is_executing_action = True
                return True
        for curr_joint_angle, cmd_joint_angle in zip(l_joint_angles, l_arm_radians):
            if abs(curr_joint_angle - cmd_joint_angle) > 0.1:
                self.is_executing_action = True
                return True
        self.is_executing_action = False
        self.is_action_received = False #only want to accept new action after current action is complete
        print("Finished executing " + str(self.action_flag) + " action")
        return False

    def load_params(self):
        with open(rp.get_path('pepper_exercise_coach') + "/config/pepper_controller_config.yaml", "r") as file:
            data = yaml.safe_load(file)
        return data

    def main(self):
        pepper_listener = Pepper()
        rate = rospy.Rate(5)  # 10hz
        # rospy.Subscriber("move_arm_command", String, pepper_listener.move_arm_callback)
        while not rospy.is_shutdown():

            #execute actions
            if self.is_executing_action:
                self.check_action_execution()
            elif self.is_action_received:
                self.execute_pepper_action()
                
            #publish camera image
            self.pub_image()

            rate.sleep()
        
        # self.shutdown_camera()

if __name__ == '__main__':
    rospy.init_node('pepper_controller', anonymous=True)
    pepper = Pepper()
    rospy.Subscriber("move_arm_command", String, pepper.move_arm_callback)
    pepper.main()