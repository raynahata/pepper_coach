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

        self.params = self.load_params()
        
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
        rospy.Subscriber("gpt_speech", String, self.gpt_callback)
        rospy.Subscriber("exercise_command", String, self.exercise_callback)


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

    def exercise_callback(self, msg):
        """
        Handles incoming exercise commands.
        """
        rospy.loginfo("Received exercise command: {}".format(msg.data))
        command = msg.data.lower()

        if command == "bicep curls":
            if not self.exercise_running:
                rospy.loginfo("Starting bicep curls...")
                self.exercise_running = True
                self.is_resting = False
                self.current_exercise = "bicep curls"
                self.bicep_curls()
                print("BICEP CURLS RUNNING")
            else:
                rospy.loginfo("Bicep curls are already running.")

        elif command == "lateral raises":
            if not self.exercise_running:
                rospy.loginfo("Starting lateral raises...")
                self.exercise_running = True
                self.is_resting = False
                self.current_exercise = "lateral raises"
                self.lateral_raises()
                print("LATERAL RAISES")
            else:
                rospy.loginfo("Lateral raises are already running.")

        elif command == "rest":
            if self.exercise_running:
                rospy.loginfo("Stopping exercise and entering rest phase...")
                self.exercise_running = False
                self.current_exercise = None
                self.is_resting = True
                self.stop_exercise_motion()
            else:
                rospy.loginfo("Already in rest phase.")

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

    def gpt_callback(self, data):
        """
        Callback for 'chat_text' topic.
        """
        rospy.loginfo("Received GPT Speech: {}".format(data.data))
        self.current_text = data.data
        self.set_flag_speaking()
        self.display_text(self.current_text)
        self.say_text(self.current_text)
        # self.display_text(self.current_text)
        self.set_flag_listening()

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

    def lateral_arm_motion_up(self):
        """
        Move both arms to the 'up' position.
        """
        # Right arm angles in degrees (arm up)
        right_arm_angles_degrees = [101.2, -89.4, 97.3, 5.8, -1.0]
        right_arm_angles_radians = self.degrees_to_radians(right_arm_angles_degrees)

        # Left arm angles in degrees (arm up)
        left_arm_angles_degrees = [101.2, 89.4, -97.3, -5.8, 1.0]
        left_arm_angles_radians = self.degrees_to_radians(left_arm_angles_degrees)

        # Move the right arm
        self.move_arm("R", right_arm_angles_radians, speed=0.2)

        # Move the left arm
        self.move_arm("L", left_arm_angles_radians, speed=0.2)

    ### Hardcoded Arm Motion: Down ###
    def lateral_arm_motion_down(self):
        """
        Move both arms to the 'down' position.
        """
        # Right arm angles in degrees (arm down)
        right_arm_angles_degrees = [101.2, -0.5, 97.3, 5.8, -1.0]
        right_arm_angles_radians = self.degrees_to_radians(right_arm_angles_degrees)

        # Left arm angles in degrees (arm down)
        left_arm_angles_degrees = [101.2, 2.3, -98, -6, 1.9]
        left_arm_angles_radians = self.degrees_to_radians(left_arm_angles_degrees)

        # Move the right arm
        self.move_arm("R", right_arm_angles_radians, speed=0.2)

        # Move the left arm
        self.move_arm("L", left_arm_angles_radians, speed=0.2)

    ### Hardcoded Arm Motion: Up ###
    def bicep_arm_motion_up(self):
        """
        Move both arms to the 'up' position.
        """
        # Right arm angles in degrees (arm up)
        right_arm_angles_degrees = [76.0, -23.0, 83.0, 89.0, 104.5]
        right_arm_angles_radians = self.degrees_to_radians(right_arm_angles_degrees)

        # Left arm angles in degrees (arm up)
        left_arm_angles_degrees = [76.0, 23.0, -83.0, -89.0, -104.5]
        left_arm_angles_radians = self.degrees_to_radians(left_arm_angles_degrees)

        # Move the right arm
        self.move_arm("R", right_arm_angles_radians, speed=0.1)

        # Move the left arm
        self.move_arm("L", left_arm_angles_radians, speed=0.1)

    ### Hardcoded Arm Motion: Down ###
    def bicep_arm_motion_down(self):
        """
        Move both arms to the 'down' position.
        """
        # Right arm angles in degrees (arm down)
        right_arm_angles_degrees = [76.0, -23.0, 83.0, 0.7, 104.5]
        right_arm_angles_radians = self.degrees_to_radians(right_arm_angles_degrees)

        # Left arm angles in degrees (arm down)
        left_arm_angles_degrees = [76.0, 23.0, -83.0, -0.7, -104.5]
        left_arm_angles_radians = self.degrees_to_radians(left_arm_angles_degrees)

        # Move the right arm
        self.move_arm("R", right_arm_angles_radians, speed=0.1)

        # Move the left arm
        self.move_arm("L", left_arm_angles_radians, speed=0.1)

    ### Looping Arm Motion ###
    def bicep_curls(self):
        """
        Moves Pepper's arms up and down for bicep curls until stopped.
        """
        rospy.loginfo("Pepper is performing bicep curls.")

        try:
            while self.exercise_running and not rospy.is_shutdown():
                self.bicep_arm_motion_up()
                rospy.loginfo("Arms moved up.")

                rospy.sleep(2)

                self.bicep_arm_motion_down()
                rospy.loginfo("Arms moved down.")
                rospy.sleep(2)

            rospy.loginfo("Bicep curls stopped. Entering rest phase.")
            self.exercise_running = False
            self.current_exercise = None
            self.is_resting = True
            self.exercise_publisher.publish("rest")

        except rospy.ROSInterruptException:
            rospy.loginfo("Bicep curls interrupted.")

    def lateral_raises(self):
        """
        Moves Pepper's arms outward for lateral raises until stopped.
        """
        rospy.loginfo("Pepper is performing lateral raises.")

        try:
            while self.exercise_running and not rospy.is_shutdown():
                self.lateral_arm_motion_up()
                rospy.loginfo("Arms moved up.")

                rospy.sleep(2)

                self.lateral_arm_motion_down()
                rospy.loginfo("Arms moved down.")
                rospy.sleep(2)

            rospy.loginfo("Lateral raises stopped. Entering rest phase.")
            self.exercise_running = False
            self.current_exercise = None
            self.is_resting = True
            self.exercise_publisher.publish("rest")

        except rospy.ROSInterruptException:
            rospy.loginfo("Lateral raises interrupted.")

    ### Begin code for Pepper's movements in response to rating/movement ###
    def firm_position_action(self):
        """
        Pepper leans forward and arms forward toward the side at a low speed
        """

        # Right arm angles in degrees (arm forward toward the side)
        right_arm_angles_degrees = [66.3, -4.8, 97.8, 6.6, -1.9]
        right_arm_angles_radians = self.degrees_to_radians(right_arm_angles_degrees)

        # Left arm angles in degrees (arm forward toward the side)
        left_arm_angles_degrees = [66.3, 4.8, -97.8, -6.6, 1.9]
        left_arm_angles_radians = self.degrees_to_radians(left_arm_angles_degrees)

        # torso angles in degrees (leaning forward)
        torso_angles_degrees = [-0.3, -32.6, -0.8]
        torso_angles_radians = self.degrees_to_radians(torso_angles_degrees)

        neck_angles_degrees = [1.1, -16.5]
        neck_angles_radians = self.degrees_to_radians(neck_angles_degrees)

        # move the right arm
        self.move_arm("R", right_arm_angles_radians, speed=0.1)

        # move the left arm
        self.move_arm("L", left_arm_angles_radians, speed=0.1)

        # move the torso
        self.move_torso(torso_angles_radians, speed=0.1)

        # move the neck
        self.move_neck(neck_angles_radians, speed=0.1)

    def neutral_position_action(self):
        """
        Pepper has torso straight, arm positions to the side
        """
        #Right arm angles in degrees (arms to the side)
        right_arm_angles_degrees = [99.1, -6.1, 97.0, 5.6, -1.9]
        right_arm_angles_radians = self.degrees_to_radians(right_arm_angles_degrees)

        # Left arm angles in degrees (arm to the side)
        left_arm_angles_degrees = [99.1, 6.1, -97.0, -5.6, 1.9]
        left_arm_angles_radians = self.degrees_to_radians(left_arm_angles_degrees)

        # torso angles in degrees (neutral)
        torso_angles_degrees = [-0.6, -2.2, -0.6]
        torso_angles_radians = self.degrees_to_radians(torso_angles_degrees)

        # neck angles in degrees (ensure that Pepper's neck is upright for pose tracking)
        neck_angles_degrees = [1.1, -19.4]
        neck_angles_radians = self.degrees_to_radians(neck_angles_degrees)

        # move the right arm
        self.move_arm("R", right_arm_angles_radians, speed=0.1)

        # move the left arm
        self.move_arm("L", left_arm_angles_radians, speed=0.1)

        # move the torso
        self.move_torso(torso_angles_radians, speed=0.1)

        # move the neck
        self.move_neck(neck_angles_radians, speed=0.1)

    def encouraging_position_action(self):
        """
        Pepper leans backwards with arms high
        """

        # right arm angle in degrees (arm high)
        right_arm_angles_degrees = [-55.1, -0.8, 97.1, 6.4, -0.8]
        right_arm_angles_radians = self.degrees_to_radians(right_arm_angles_degrees)

        # left arm angles in degrees (arm high)
        left_arm_angles_degrees = [-55.9, 4.8, -98.2, -6.4, 1.9]
        left_arm_angles_radians = self.degrees_to_radians(left_arm_angles_degrees)

        # torso angles in degrees (leaning forward)
        torso_angles_degrees = [1.5, 15.0, -0.6]
        torso_angles_radians = self.degrees_to_radians(torso_angles_degrees)

        # move the right arm
        self.move_arm("R", right_arm_angles_radians, speed=0.1)

        # move the left arm
        self.move_arm("L", left_arm_angles_radians, speed=0.1)

        # move the torso
        self.move_torso(torso_angles_radians, speed=0.1)

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
            all_positions = self.params['feedback_position_info']['firm_positions']
            self.parse_all_movements(all_positions)
            self.is_executing_action = True
        elif self.action_flag == "encouraging":
            rospy.loginfo("Going to encouraging position")
            #self.encouraging_position_action()
            all_positions = self.params['feedback_position_info']['encouraging_positions']
            self.parse_all_movements(all_positions)
            self.is_executing_action = True
        elif self.action_flag == "neutral":
            rospy.loginfo("Going to neutral position")
            # self.neutral_position_action()
            all_positions = self.params['feedback_position_info']['neutral_positions']
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
            r_arm_degrees = self.params['feedback_position_info']['firm_positions'][r_arm_idx][degrees_idx]
            r_arm_radians = self.degrees_to_radians(r_arm_degrees)
            l_arm_degrees = self.params['feedback_position_info']['firm_positions'][l_arm_idx][degrees_idx]
            l_arm_radians = self.degrees_to_radians(l_arm_degrees)
        elif self.action_flag == "encouraging":
            r_arm_degrees = self.params['feedback_position_info']['encouraging_positions'][r_arm_idx][degrees_idx]
            r_arm_radians = self.degrees_to_radians(r_arm_degrees)
            l_arm_degrees = self.params['feedback_position_info']['encouraging_positions'][l_arm_idx][degrees_idx]
            l_arm_radians = self.degrees_to_radians(l_arm_degrees)
        #iterate through each and determine if current value close to actual
        for curr_joint_angle, cmd_joint_angle in zip(r_joint_angles, r_arm_radians):
            if abs(curr_joint_angle - cmd_joint_angle) > 0.05:
                self.is_executing_action = True
                return True
        for curr_joint_angle, cmd_joint_angle in zip(l_joint_angles, l_arm_radians):
            if abs(curr_joint_angle - cmd_joint_angle) > 0.05:
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
        
        self.shutdown_camera()

if __name__ == '__main__':
    rospy.init_node('pepper_controller', anonymous=True)
    pepper = Pepper()
    rospy.Subscriber("move_arm_command", String, pepper.move_arm_callback)
    pepper.main()