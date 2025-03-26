#!/usr/bin/env python3

#general imports
import sys
import yaml
import time
import logging
import syllables
import numpy as np
import multiprocessing
from scipy import signal
from pytz import timezone
from fastdtw import fastdtw
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
from scipy.spatial.distance import euclidean

#ros imports
import rospy
from rospkg import RosPack
from std_msgs.msg import Float64MultiArray, String, Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rp = RosPack()
np.set_printoptions(suppress=True)

class ExerciseController:
    def __init__(self, replay, log_filename, style, resting_hr, max_hr, user_id):
        #Set initial parameters
        self.replay = replay
        self.flag = False
        self.verbal_cadence = 2
        self.nonverbal_cadence = 2
        self.intercept = 0.6477586140350873
        self.slope = 0.31077594
        self.resting_hr = resting_hr
        self.max_hr = max_hr
        self.joint_comparison_idx = []
        self.computer_params = self.load_params()
        #Create data storage
        self.curr_set = ""
        self.curr_exercise = ""
        self.set_data_dict = dict()
        self.angles, self.heart_rates, self.hrr, self.all_heart_rates = [], [], [], [0]
        self.performance, self.peaks, self.feedback, self.times  = [], [], [], []
        self.current_exercise, self.exercise_name_list = '', []
        self.message_log, self.message_time_stamps = [], []
        self.eval_case_log, self.speed_case_log, self.resampled_reps = [], [], []
        self.contexts, self.actions, self.rewards  = [], [], []
        #initialize logging
        self.logger = logging.getLogger('logging')
        self.logger.setLevel(logging.DEBUG)
        fh = logging.FileHandler(rp.get_path('pepper_exercise_coach') + '/logs/{}'.format(log_filename))
        fh.setLevel(logging.DEBUG)
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)
        self.logger.addHandler(fh)
        self.logger.addHandler(ch)
        # Robot style 0 - very firm, 1 - firm, 2 - neutral, 3 - encouraging, 4 - very encouraging
        self.update_robot_style(style)
        self.init_pubs_subs()
        
    def load_params(self):
        with open(rp.get_path('pepper_exercise_coach')+"/config/computer_config.yaml", "r") as file:
            data = yaml.safe_load(file)
        return data
    
    def update_robot_style(self, robot_style):
        if robot_style == 5:
            self.robot_style = 3
        else:
            self.robot_style = robot_style
        self.neutral_expression = self.computer_params['neutral_expressions'][self.robot_style]
        self.neutral_posture = self.computer_params['neutral_postures'][self.robot_style]
        self.start_set_smile = self.computer_params['start_set_smile'][self.robot_style]
        if robot_style == 5: #adaptive
            self.adaptive = True
            ##TODO: test this with Rayna
            # self.process = subprocess.Popen(['python3.9', '-u', 'src/quori_exercises/exercise_session/adaptive_controller.py'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        else:
            self.adaptive = False
        
    def heart_rate_callback(self, hr_message):
        self.all_heart_rates.append(hr_message.data)
    
    def pose_callback(self, angle_message):
        if self.curr_set == '':
            return
        
        #grab data from message
        next_joint_angles = list(angle_message.data)
        next_timestamp = rospy.Time.now().to_sec()

        #write data to file
        next_timestamp_joint_angles = next_joint_angles.copy()
        next_timestamp_joint_angles.append(next_timestamp)

        #check if we have enough data to use for comparison
        if len(self.set_data_dict[self.curr_set]['joint_angles']) < 2:
            self.set_data_dict[self.curr_set]['joint_angles'].append(next_timestamp_joint_angles)
            return

        #grab joint data from current (most recent) and previous timestamp
        curr_timestamp_joint_angles = self.set_data_dict[self.curr_set]['joint_angles'][-1]
        prev_timestamp_joint_angles = self.set_data_dict[self.curr_set]['joint_angles'][-2]

        #grab relevant angles we want to compare
        joint_compare_idx = self.set_data_dict[self.curr_set]['angle_compare_idx']
        prev_joint_angles = [prev_timestamp_joint_angles[idx] for idx in joint_compare_idx]
        curr_joint_angles = [curr_timestamp_joint_angles[idx] for idx in joint_compare_idx]
        next_joint_angles = [next_timestamp_joint_angles[idx] for idx in joint_compare_idx]
        #grab timestamps we want to compare
        prev_timestamp = prev_timestamp_joint_angles[-1]
        curr_timestamp = curr_timestamp_joint_angles[-1]
        next_timestamp = next_timestamp_joint_angles[-1]

        # #calculate slope between prev and curr timestamps
        prev_joint_deriv = [(curr_joint_angles[idx] - prev_joint_angles[idx]) / (curr_timestamp - prev_timestamp)
            for idx in range(len(prev_joint_angles))]
        #calculate slopes between curr and next timestamps
        next_joint_deriv = [(curr_joint_angles[idx] - next_joint_angles[idx]) / (curr_timestamp - next_timestamp)
            for idx in range(len(next_joint_angles))]
        
        #grab values from config for exercise
        exercise_min = self.computer_params['exercise_info'][self.curr_exercise]['current_angles_min']
        exercise_grad = self.computer_params['exercise_info'][self.curr_exercise]['max_grad']

        #check for change in slope across prev and next 
        for prev_deriv, next_deriv in zip(prev_joint_deriv, next_joint_deriv):
            #exclude noise in slope measurements
            if abs(prev_deriv) < 5 and abs(next_deriv) < 5:
                continue
            #check if we're at a peak
            is_peak = True if (prev_deriv * next_deriv) < 0 else False
            #check if arms are raised
            is_arm_raised = True if (prev_deriv) > exercise_grad else False #also next grad?
            #check if min joint angle reached
            is_min_angle_reached = True if np.min(prev_joint_angles) < exercise_min or \
                np.min(curr_joint_angles) < exercise_min or np.min(next_joint_angles) < exercise_min else False
            #ensure that we don't have duplicate readings
            if len(self.set_data_dict[self.curr_set]['peaks']) > 0:
                timestamp_threshold = 1.0 #atleast 1 second between subsequent measurements
                prev_peak_timestamp = self.set_data_dict[self.curr_set]['peaks'][-1]
                delta_timestep = curr_timestamp - prev_peak_timestamp
                is_new_peak = True if (delta_timestep > timestamp_threshold) else False
            else:
                delta_timestep = -1
                is_new_peak = True
            #combine conditional statements
            if is_peak and is_arm_raised and is_min_angle_reached and is_new_peak:
                print("NEW PEAK DETECTED", prev_deriv, delta_timestep)
                self.set_data_dict[self.curr_set]['peaks'].append(curr_timestamp)
        #append current data to dictionary
        self.set_data_dict[self.curr_set]['joint_angles'].append(next_timestamp_joint_angles)

    def init_pubs_subs(self):
        if not self.replay:
            #publishers
            self.sound_pub = rospy.Publisher("quori_sound", String, queue_size=10)
            self.movement_pub = rospy.Publisher('quori/joint_trajectory_controller/command', JointTrajectory, queue_size=10)
            self.emotion_pub = rospy.Publisher('quori/face_generator_emotion', Float64MultiArray, queue_size=10)
            #subscribers
            self.heart_rate_sub = rospy.Subscriber("/heart_rate", Int32, self.heart_rate_callback, queue_size=3000)
            self.pose_sub = rospy.Subscriber("/joint_angles", Float64MultiArray, self.pose_callback, queue_size=3000)
    
    def message(self, m, priority=2):
        #Only message if it has been 3 sec since last message ended
        if (len(self.message_time_stamps)) > 0:
            last_message_time = self.message_time_stamps[-1]
            if (datetime.now(timezone('EST')) - last_message_time).total_seconds() < 3 and priority < 2:
                #Skip message
                self.logger.info('Skipping {}'.format(m))
                return False
                
        self.logger.info('Robot says: {}'.format(m))
        length_estimate = np.round(self.slope*syllables.estimate(m) + self.intercept)

        if not self.replay:
            self.sound_pub.publish(m)

        self.message_log.append(m)
        self.message_time_stamps.append(datetime.now(timezone('EST')) + timedelta(seconds=length_estimate) )

        return True

    def send_body(self, start_position, end_position, duration):
        if not self.replay:
            #Start point
            traj = JointTrajectory()
            traj.joint_names = ["r_shoulder_pitch", "r_shoulder_roll", "l_shoulder_pitch", "l_shoulder_roll", "waist_pitch"]
            point_1 = JointTrajectoryPoint()
            point_1.time_from_start = rospy.Duration(duration / 2)
            point_1.positions = start_position
            traj.points=[point_1]
            self.movement_pub.publish(traj)

            time.sleep(duration/2)

            #End point
            traj = JointTrajectory()
            traj.joint_names = ["r_shoulder_pitch", "r_shoulder_roll", "l_shoulder_pitch", "l_shoulder_roll", "waist_pitch"]

            point_2 = JointTrajectoryPoint()
            point_2.time_from_start = rospy.Duration(duration / 2)
            point_2.positions = end_position
            traj.points=[point_2]
            self.movement_pub.publish(traj)

    def move_right_arm(self, start, end):
        #["r_shoulder_pitch", "r_shoulder_roll", "l_shoulder_pitch", "l_shoulder_roll", "waist_pitch"]
        arm_halfway = list(self.neutral_posture)
        arm_halfway[0] = 1.1
        
        arm_sides = list(self.neutral_posture)
        
        arm_up = list(self.neutral_posture)
        arm_up[0] = 1.7
        
        positions = {'halfway': arm_halfway, 'sides': arm_sides, 'up': arm_up}
        
        self.logger.info('Moving arm from {} to {}'.format(start, end))
        
        self.send_body(positions[start], positions[end], 4)

    def start_new_set(self, exercise_name, set_num, tot_sets):
        self.curr_set = 'set_'+str(set_num)
        self.curr_exercise = exercise_name
        #initialize new set in dictionary
        self.set_data_dict[self.curr_set] = dict()
        self.set_data_dict[self.curr_set]['joint_angles'] = []
        #add exercise name to dictionary
        self.set_data_dict[self.curr_set]['exercise_name'] = exercise_name
        #add peaks list to dictionary
        self.set_data_dict[self.curr_set]['peaks'] = []
        #specify angles to compare for pose callback
        self.set_data_dict[self.curr_set]['angle_compare_idx'] = []
        for group, ind in self.computer_params['exercise_info'][exercise_name]['segmenting_joints']:
            self.set_data_dict[self.curr_set]['angle_compare_idx'].append(
                self.computer_params['angle_order'][group][ind]
            )

        # dummy_angle_data = np.zeros((0,len(self.computer_params['angle_info'])*3))
        # self.angles.append(dummy_angle_data)
        # self.heart_rates.append([])
        # self.hrr.append([])
        # self.performance.append(np.empty((0, len(self.computer_params['exercise_info'][exercise_name]['comparison_joints']))))
        # self.peaks.append([])
        # self.feedback.append([])
        # self.times.append([])
        # self.current_exercise = exercise_name
        # self.exercise_name_list.append(exercise_name)
        # self.eval_case_log.append([])
        # self.speed_case_log.append([])

        #Raise arm all the way up
        self.move_right_arm('up', 'halfway')

        #Add starting set to log
        self.logger.info('=====================================')
        self.logger.info('STARTING SET {} OF {} - {}'.format(set_num, tot_sets, self.current_exercise))
        self.logger.info('=====================================')

        #Robot says starting set and smile
        robot_message = "Get ready to start set %s of %s. You will be doing %s." % (set_num, tot_sets, exercise_name.replace("_", " " ))
        self.message(robot_message)
        # self.change_expression('smile', self.start_set_smile, 4) ##TODO: change to smile based command for pepper robot


##MAIN FUNCTION FOR STANDALONE EXERCISE CONTROLLER
# if __name__ == '__main__':
    
#     PARTICIPANT_ID = '0'
#     ROBOT_STYLE = 5 #1 is firm, 3 is encouraging, 5 is adaptive
#     RESTING_HR = 97
#     MAX_HR = 220-26
#     LOG_FILENAME = "test.pickle"

#     rospy.init_node('exercise_controller_node')

#     controller = ExerciseController(False, LOG_FILENAME, ROBOT_STYLE, RESTING_HR, MAX_HR, PARTICIPANT_ID)
#     rospy.sleep(2.0)

#     controller.start_new_set(exercise_name='bicep_curls',set_num=1, tot_sets=1)

#     rospy.spin()

