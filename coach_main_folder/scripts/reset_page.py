# export PYTHONPATH=${PYTHONPATH}:/home/raynahata/exercise_bot/pynaoqi-python2.7-2.8.6.23-linux64-20191127_152327/lib/python2.7/site-packages

import rospy
from naoqi import ALProxy

class PepperWebDisplay:
    def __init__(self):
        self.IP = "128.237.236.27"  # Replace with Pepper's actual IP
        self.tablet = ALProxy("ALTabletService", self.IP, 9559)

        rospy.init_node('pepper_web_display', anonymous=True)
        rospy.loginfo("Displaying CMU website on Pepper's tablet.")
        
        # Automatically show CMU website
        self.display_webpage("https://www.cmu.edu")

    def display_webpage(self, url):
        """
        Displays a webpage on Pepper's tablet.
        """
        try:
            self.tablet.showWebview(url)
            rospy.loginfo("Successfully displayed webpage: {}".format(url))
        except Exception as e:
            rospy.logerr("Failed to display webpage: {}".format(e))

    def clear_webpage(self):
        """
        Clears the webpage from Pepper's screen.
        """
        try:
            self.tablet.hideWebview()
            rospy.loginfo("Cleared webpage from Pepper's tablet.")
        except Exception as e:
            rospy.logerr("Failed to clear webpage: {}".format(e))

def main():
    rospy.init_node('pepper_web_display', anonymous=True)
    PepperWebDisplay()  # Automatically shows CMU website

    try:
        rospy.spin()  # Keep the node running
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Pepper Web Display.")

if __name__ == '__main__':
    main()