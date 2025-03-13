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
        self.computer_params = self.load_params()
        #Create data storage
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
        if len(self.angles) == 0 or len(self.peaks) == 0 or not self.flag:
            return

        #Read angle from message
        callback_data = angle_message.data
        callback_data = np.array(callback_data)
        angle = callback_data[:-1]
        self.angles[-1] = np.vstack((self.angles[-1], np.array(angle)))

        #Save heart rate and hrr
        self.heart_rates[-1].append(self.all_heart_rates[-1])
        hrr = (self.all_heart_rates[-1] - self.resting_hr) / float(self.max_hr - self.resting_hr)
        self.hrr[-1].append(hrr)

        #Save the time of the message
        current_time = callback_data[-1]
        self.times[-1].append(current_time)

        #Get time of last peak and time of last angle
        last_peak_time = (self.times[-1][self.peaks[-1][-1]] if len(self.peaks[-1]) > 0 else 0) - self.times[-1][0]
        last_angle_time = current_time - self.times[-1][0]

        #Look for new peaks after 1 second and every 5 angles
        if last_angle_time > 1 and self.angles[-1].shape[0] % 5:
            # print('Condition 1: Last Peak Time {}, Last Angle Time {}'.format(last_peak_time, last_angle_time))

            #Check if far enough away from previous peak
            if len(self.peaks[-1]) == 0 or (last_peak_time + 0.75 < last_angle_time):
                curr_exercise_info = self.computer_params['exercise_info'][self.current_exercise]
                
                #Interval to check for peaks
                to_check_amount = 10
                to_check_times = self.times[-1][-to_check_amount:] - np.array(self.times[-1][0])

                #Calculate maxes and mins in that interval
                current_angles_min = np.min(self.angles[-1][-to_check_amount:,:][:,curr_exercise_info['segmenting_joint_inds']])
                current_angles_max = np.max(self.angles[-1][-to_check_amount:,:][:,curr_exercise_info['segmenting_joint_inds']])

                #Calculate max gradients in interval
                grad = []
                for index in curr_exercise_info['segmenting_joint_inds']:
                    grad.append(np.max(np.gradient(self.angles[-1][-to_check_amount:,][:,index], to_check_times)))
                
                #Calculate max gradient
                max_grad = np.max(grad)
                max_val_pos = np.argmax(self.angles[-1][-to_check_amount:,:][:,curr_exercise_info['segmenting_joint_inds']])
                min_val_pos = np.argmin(self.angles[-1][-to_check_amount:,:][:,curr_exercise_info['segmenting_joint_inds']])

                # print('Condition 2: Shape {}, Current Min {}, Current Max {}, Max Grad {}'.format(self.angles[-1].shape[0], current_angles_min, current_angles_max, max_grad))
                peak_to_add = None
                
                if current_angles_min < curr_exercise_info['current_angles_min'] and max_grad > curr_exercise_info['max_grad']:
                    
                    peak_candidate = self.angles[-1].shape[0]-to_check_amount+min_val_pos-1

                    peak_candidate = np.min([self.angles[-1].shape[0]-1, peak_candidate])
                    
                    peak_candidate_time = self.times[-1][peak_candidate] - self.times[-1][0]

                    #Make sure peak candidate is not too close to previous peak and that a max has been reached between previous peak and current candidate
                    if len(self.peaks[-1]) == 0 or (last_peak_time + 0.75 < peak_candidate_time and np.max(self.angles[-1][self.peaks[-1][-1]:peak_candidate,:][:,curr_exercise_info['segmenting_joint_inds']]) > curr_exercise_info['max_in_range']):
                        # print('Condition 3: Peak Candidate {}, Peak Candidate Time {}'.format(peak_candidate, peak_candidate_time))
                        if self.current_exercise == 'bicep_curls':
                            if np.min(self.angles[-1][:,curr_exercise_info['segmenting_joint_inds']][peak_candidate,:]) > 100:
                                peak_to_add = peak_candidate
                        
                        if self.current_exercise == 'lateral_raises':
                            if np.max(self.angles[-1][:,curr_exercise_info['segmenting_joint_inds']][peak_candidate,:]) < 40:
                                peak_to_add = peak_candidate

                if peak_to_add:

                    self.peaks[-1].append(peak_candidate)
                            
                    self.logger.info('---Peak detected - {}'.format(self.peaks[-1][-1]))

                    #Evaluate new rep
                    if len(self.peaks[-1]) > 1:
                        rep_duration = (self.times[-1][self.peaks[-1][-1]] - self.times[-1][self.peaks[-1][-2]])
                        start = time.time()
                        self.evaluate_rep(self.peaks[-1][-2], self.peaks[-1][-1], rep_duration)
                        end = time.time()
                        self.logger.info('Evaluation took {} seconds'.format(np.round(end-start, 1)))

        else:
            #Check if no movement in the last few seconds
            if last_angle_time > 10:
                min_in_range = np.min(self.angles[-1][-30:,:][:,curr_exercise_info['segmenting_joint_inds'][0]])
                max_in_range = np.max(self.angles[-1][-30:,:][:,curr_exercise_info['segmenting_joint_inds'][0]])
                
                if (max_in_range - min_in_range) < 10:
                    #Get style specific message
                    _, m = self.get_message(['no movement'])
                    self.message(m, priority=1)
                
                #If peaks are too far apart, say something as a filler
                elif (len(self.peaks[-1]) == 0 and last_angle_time > 10) or (last_angle_time - last_peak_time > 15):
                    #Get style specific message
                    _, m = self.get_message(['peaks far apart'])
                    self.message(m, priority=1)

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
        #Update data storage
        self.angles.append(np.empty((0, len(self.computer_params['angle_info'])*3)))
        self.heart_rates.append([])
        self.hrr.append([])
        self.performance.append(np.empty((0, len(self.computer_params['exercise_info'][exercise_name]['comparison_joints']))))
        self.peaks.append([])
        self.feedback.append([])
        self.times.append([])
        self.current_exercise = exercise_name
        self.exercise_name_list.append(exercise_name)
        self.eval_case_log.append([])
        self.speed_case_log.append([])

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
