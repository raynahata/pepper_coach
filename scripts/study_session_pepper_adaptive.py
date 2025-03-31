#!/usr/bin/env python3

#general imports
import time
import pickle
import logging
import numpy as np
from pytz import timezone
from pynput import keyboard ##fix this import
from datetime import datetime
import matplotlib.pyplot as plt

#ros imports
import rospy
import rosbag
from rospkg import RosPack
from std_msgs.msg import Int32, String

#package imports
from exercise_manager import ExerciseManager

#params TOOD: move these to yaml file
SET_LENGTH = 40
REST_TIME = 40
EXERCISE_LIST = ['bicep_curls'] #comment out before actual sessions
# EXERCISE_LIST = ['bicep_curls', 'bicep_curls', 'lateral_raises', 'lateral_raises']

#Change at beginning of study - make sure to change in adaptive_controller.py as well
PARTICIPANT_ID = '0'
RESTING_HR = 97
AGE = 26

#Change at beginning of each round
ROBOT_STYLE = 5 #1 is firm, 3 is encouraging, 5 is adaptive
ROUND_NUM = 1 #0 is intro

MAX_HR = 220-AGE

VERBAL_CADENCE = 2 #1 is low, 2 is medium, 3 is high
NONVERBAL_CADENCE = 2

rp = RosPack()

class StudySession:
    def __init__(self):
        #init data storage
        self.intake_heart_rates = []
        #init filenames
        self.log_filename = 'Participant_{}_Style_{}_Round_{}_{}.log'.format(PARTICIPANT_ID, ROBOT_STYLE, ROUND_NUM, datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))
        self.data_filename = 'Participant_{}_Style_{}_Round_{}_{}.pickle'.format(PARTICIPANT_ID, ROBOT_STYLE, ROUND_NUM, datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))

    def intake_heart_rate_callback(self, msg):
        self.intake_heart_rates.append(msg.data)

    def main(self):
        rospy.init_node('pepper_study_session', anonymous=True)
        rate = rospy.Rate(10)

        #publisher
        pepper_action_pub = rospy.Publisher("/pepper_action_request", String, queue_size=10)

        #subscribers
        heart_rate_sub = rospy.Subscriber("/heart_rate", Int32, self.intake_heart_rate_callback, queue_size=3000)

        #init controller
        controller = ExerciseManager(False, self.log_filename, ROBOT_STYLE, RESTING_HR, MAX_HR, PARTICIPANT_ID)
        rospy.sleep(5)

        #Note from refactor: no round 0. All rounds should be > 0
        input("Press Enter to to start exercise session...")
        controller.message('Let us start Round {} now. Please stand in the blue square and pick up the dumbbells if you want to use them'.format(ROUND_NUM))
        input("Press Enter to to start exercise session...")

        #For each exercise
        for set_num, exercise_name in enumerate(EXERCISE_LIST):
                    
            #Start a new set
            controller.start_new_set(exercise_name, set_num+1, len(EXERCISE_LIST))
            
            controller.logger.info('-------------------Recording!')
            start_message = False
            halfway_message = False

            #publish neutral action to pepper
            pepper_action = 'positive_neutral'
            pepper_action_pub.publish(pepper_action)
            
            inittime = datetime.now(timezone('EST'))
            
            #Stop between minimum and maximum time and minimum reps
            while (datetime.now(timezone('EST')) - inittime).total_seconds() < SET_LENGTH:        
                        
                #Robot says starting set
                if not start_message:
                    robot_message = "Start %s now" % (exercise_name.replace("_", " " ))
                    controller.message(robot_message)
                    start_message = True

                controller.flag = True

                if (datetime.now(timezone('EST')) - inittime).total_seconds() > SET_LENGTH/2 and not halfway_message:
                    robot_message = "You are halfway"
                    controller.message(robot_message)
                    halfway_message = True 

                if (datetime.now(timezone('EST')) - inittime).total_seconds() > SET_LENGTH:
                    break 

            controller.flag = False
            controller.logger.info('-------------------Done with exercise')

            robot_message = "Almost done."
            controller.message(robot_message)
            rospy.sleep(3)

            rest_start = datetime.now(timezone('EST'))

            robot_message = "Time to rest."
            controller.message(robot_message)

            #publish neutral action to pepper
            pepper_action = 'positive_neutral'
            pepper_action_pub.publish(pepper_action)
            
            if set_num + 1 < len(EXERCISE_LIST):
                halfway_message = False
                while (datetime.now(timezone('EST')) - rest_start).total_seconds() < REST_TIME:
                    
                    #Print halfway done with rest here
                    if (datetime.now(timezone('EST')) - rest_start).total_seconds() > REST_TIME/2 and not halfway_message:
                        halfway_message = True
                        robot_message = "Rest for {} more seconds.".format(int(REST_TIME/2))
                        controller.message(robot_message)
            else:
                robot_message = "Round complete. Please take a seat in the chair and complete a survey about this round on the laptop next to you."
                controller.message(robot_message)

        if controller.robot_style == 5:
            controller.process.stdin.write('exit\n')
            controller.process.stdin.flush()

            if controller.process.stdin:
                controller.process.stdin.close()
            if controller.process.stdout:
                controller.process.stdout.close()

        #dump dictionary into pickle
        with open(rp.get_path('pepper_exercise_coach') + '/data/{}'.format(self.data_filename), 'wb') as f:
            pickle.dump(controller.set_data_dict, f)

        controller.logger.info('Saved file {}'.format(self.data_filename))
        controller.logger.handlers.clear()
        logging.shutdown()
        print('Done!')

if __name__ == '__main__':
    session = StudySession()
    session.main()
    

##TODO: record video from user