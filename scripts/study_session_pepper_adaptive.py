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
from std_msgs.msg import Int32

#package imports
from exercise_manager import ExerciseManager

#params TOOD: move these to yaml file
SET_LENGTH = 20
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

            #Lower arm all the way down #TODO: publish to pepper topic
            #controller.move_right_arm('halfway', 'sides')
            #from lateral_arm_motion_down
            # right_arm_angles_degrees = [101.2, -0.5, 97.3, 5.8, -1.0]
            # right_arm_angles_radians = pepper_controller.degrees_to_radians(right_arm_angles_degrees)
            # pepper_controller.move_arm("R",  right_arm_angles_radians, speed=0.2)
            
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
            # controller.change_expression('smile', controller.start_set_smile, 4) ##TODO:change to pepper smile based command

            #Raise arm all the way up #TODO: publish to pepper topic
            # controller.move_right_arm('sides', 'up')
            # from lateral_arm_motion_up
            # right_arm_angles_degrees = [101.2, -89.4, 97.3, 5.8, -1.0]
            # right_arm_angles_radians = pepper_controller.degrees_to_radians(right_arm_angles_degrees)
            # pepper_controller.move_arm("R", right_arm_angles_radians, speed=0.2)
            
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

        # controller.change_expression('smile', controller.start_set_smile, 4) ##TODO:change to pepper smile based command

        if controller.robot_style == 5:
            controller.process.stdin.write('exit\n')
            controller.process.stdin.flush()

            if controller.process.stdin:
                controller.process.stdin.close()
            if controller.process.stdout:
                controller.process.stdout.close()

        data = {'angles': controller.angles, 'peaks': controller.peaks, 'feedback': controller.feedback, 'times': controller.times, 'exercise_names': controller.exercise_name_list, 'all_hr': controller.all_heart_rates, 'heart_rates': controller.heart_rates, 'hrr': controller.hrr, 'actions': controller.actions, 'context': controller.contexts, 'rewards': controller.rewards}
        
        dbfile = open(rp.get_path('pepper_exercise_coach') + '/data/{}'.format(self.data_filename), 'ab')

        pickle.dump(data, dbfile)                    
        dbfile.close()

        controller.logger.info('Saved file {}'.format(self.data_filename))
        controller.logger.handlers.clear()
        logging.shutdown()
        print('Done!')

        # controller.plot_angles() ##TODO: need to copy over when I have actual data
        # plt.show()


if __name__ == '__main__':
    session = StudySession()
    session.main()
    