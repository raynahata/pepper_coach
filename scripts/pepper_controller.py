import sys
from naoqi import ALProxy

import rospy
import math
import time
from std_msgs.msg import String

# Replace this with your Pepper's IP address
pepper_ip = "128.237.236.27"
pepper_port = 9559
action_received = False


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

        self.state = ""
        self.current_text = ""
        self.action_flag = ""

        # ROS Publishers and Subscribers
        rospy.init_node("pepper_controller", anonymous=True)
        self.state_pub = rospy.Publisher("pepper_state", String, queue_size=10)
        self.text_pub = rospy.Publisher("chat_text", String, queue_size=10)
        self.exercise_publisher = rospy.Publisher("/exercise_command", String, queue_size=10)
        rospy.Subscriber("pepper_state", String, self.callback_state)
        rospy.Subscriber("gpt_speech", String, self.gpt_callback)
        rospy.Subscriber("exercise_command", String, self.exercise_callback)
        self.angle_publisher = rospy.Publisher("arm_angles", String, queue_size=10)

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
        parsed_angles = ' '.join("" + angle for angle in angles)
        self.angle_publisher.publish(parsed_angles)

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

        rospy.loginfo("Moving {} arm to angles: {}".format(side, angles))
        self.motion.setAngles(joint_names, angles, speed)
        # self.motion.angleInterpolation(joint_names,angles,[speed]*len(joint_names),True)

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

        rospy.loginfo("Moving torso to angles: {}".format(angles))
        # TODO: confirm that the joint_names below make sense
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
            else:
                rospy.loginfo("Bicep curls are already running.")

        elif command == "lateral raises":
            if not self.exercise_running:
                rospy.loginfo("Starting lateral raises...")
                self.exercise_running = True
                self.is_resting = False
                self.current_exercise = "lateral raises"
                self.lateral_raises()
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

        //TODO: remove this when you're done (note to self below)
            joint_names = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
            joint_names = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
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

        # move the right arm
        self.move_arm("R", right_arm_angles_radians, speed=0.1)

        # move the left arm
        self.move_arm("L", left_arm_angles_radians, speed=0.1)

        # move the torso
        self.move_torso(torso_angles_radians, speed=0.1)

    def neutral_position_action(self):
        """
        Pepper has torso straight, arm positions move in somewhat random motion
        //TODO: implement this. For now, just do a completely neutral position (no random movements)
        """
        return True

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

        if command == "firm":
            self.action_flag = "firm"

        elif command == "encouraging":
            self.action_flag = "encouraging"

    def listener(self):
        """
        Start the ROS listener node and execute the arm motion loop.
        """
        rospy.loginfo("Starting listener...")
        self.move_arms_up_and_down()

    def main(self):
        # rospy.init_node('pepper_controller', anonymous=True)
        pepper_listener = Pepper()
        rate = rospy.Rate(10)  # 10hz
        # rospy.Subscriber("move_arm_command", String, pepper_listener.move_arm_callback)

        #rospy.spin()  # Keep the node running
        while not rospy.is_shutdown():
            if self.action_flag == "firm":
                action_received = True
                rospy.loginfo("Going to firm position")
                pepper_listener.firm_position_action()

            elif self.action_flag == "encouraging":
                action_received = True
                rospy.loginfo("Going to encouraging position")
                pepper_listener.encouraging_position_action()
            else: action_received = False
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pepper_controller', anonymous=True)
    pepper = Pepper()
    rospy.Subscriber("move_arm_command", String, pepper.move_arm_callback)
    pepper.main()


    # rospy.init_node('pepper_controller', anonymous=True)
    # pepper = Pepper()
    # pepper.clear_screen()
    #
    # pepper.firm_position_action()
    # rospy.loginfo("Firm position executed.")
    # rospy.sleep(15)
    #
    # pepper.stop_exercise_motion()
    # rospy.sleep(15)
    # rospy.loginfo("Pepper returned to neutral position.")
    #
    # pepper.encouraging_position_action()
    # rospy.loginfo("Encouraging position executed.")
    # rospy.sleep(15)
    #
    # rospy.loginfo("Two feedback positions tested: firm and encouraging")

    # try:
    #     rospy.spin()  # Keep the node running
    # except KeyboardInterrupt:
    #     rospy.loginfo("Shutting down Pepper Listener.")

# try:
#     print('slay queen ')
#     # Create a proxy to ALMotion
#     motion_proxy = ALProxy("ALMotion", pepper_ip, pepper_port)
#
#     # Wake up Pepper (set to stand posture)
#     motion_proxy.wakeUp()
#
#     pepper = Pepper()
#     pepper.firm_position_action()
#
#     # After finishing movements, let Pepper rest
#     motion_proxy.rest()
#
# except Exception as e:
#     print("Error: ", e)
