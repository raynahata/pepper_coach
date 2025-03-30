#!/usr/bin/env python3

#general imports
import sys
import json
import yaml
import time
import pickle
import logging
import syllables
import subprocess
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

class ExerciseManager:
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
        self.exercise_params = self.load_params()
        self.experts = self.load_experts()
        self.pepper_messages = self.load_messages()
        #Create data storage
        self.curr_set = ""
        self.curr_exercise = ""
        self.set_data_dict = dict()
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
    
    def load_experts(self):
        # load expert file from directory
        with open(rp.get_path('pepper_exercise_coach') + "/config/new_experts_smaller.pickle", "rb") as file:
            expert_data = pickle.load(file)
        #iterate through data and add duration information
        for exercise in expert_data:
            candidates = []
            for rep in expert_data[exercise]['good']:
                candidates.append(rep[-1,0] - rep[0, 0])
            if len(candidates) > 0:
                expert_data[exercise]['average duration'] = np.mean(candidates)
            else:
                expert_data[exercise]['average duration'] = 4
        return expert_data
    
    def load_messages(self):
        all_messages = dict()
        for e in ['bicep curls', 'lateral raises']:
            all_messages[e] = dict()
            for f in ['low', 'moderate', 'high']:
                with open(rp.get_path('pepper_exercise_coach') + "/config/{}_{}_edited.json".format(e, f)) as file:
                    all_messages[e][f] = json.load(file)
        return all_messages

    def update_robot_style(self, robot_style):
        if robot_style == 5:
            self.robot_style = 3
        else:
            self.robot_style = robot_style
        self.neutral_expression = self.exercise_params['neutral_expressions'][self.robot_style]
        self.neutral_posture = self.exercise_params['neutral_postures'][self.robot_style]
        self.start_set_smile = self.exercise_params['start_set_smile'][self.robot_style]
        if robot_style == 5: #adaptive
            self.adaptive = True
            self.process = subprocess.Popen(['python3.9', '-u', rp.get_path('pepper_exercise_coach') + '/scripts/adaptive_controller.py'], 
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        else:
            self.adaptive = False
        
    def heart_rate_callback(self, hr_message):
        if self.curr_set != '':
            self.set_data_dict[self.curr_set]['all_heart_rates'].append(hr_message.data)
    
    def calc_dist_worker(self, ind, series1, series2, q):
        q.put((ind, fastdtw(series1, series2, dist=euclidean)))
    
    def calc_dist(self, start_idx, indices, labels):
        input_values = []
        label_values = []
        to_choose = 3
        for label in labels:
            experts_to_add = self.experts[self.curr_exercise][label]
            choose = np.random.choice(len(experts_to_add), np.min([len(experts_to_add), to_choose])).astype('int')
            for ii in choose:
                series2 = experts_to_add[ii][:, 1:]
                series2 = series2[:, indices]
                input_values.append(series2)
            label_values.extend([label]*len(choose))
        
        qout = multiprocessing.Queue()

        series1 = np.array(self.set_data_dict[self.curr_set]['joint_angles'])
        series1 = series1[start_idx:, indices]

        processes = [multiprocessing.Process(target=self.calc_dist_worker, args=(ind, series1, val, qout))
                for ind, val in enumerate(input_values)]
        
        for p in processes:
            p.start()

        for p in processes:
            p.join()
        
        unsorted_result = [qout.get() for p in processes]
        result = [t[1][0] for t in sorted(unsorted_result)]

        return result, label_values
    
    def get_bad_eval_cases(self, feedback):
        c = []

        num_bad_in_row = 4 - self.verbal_cadence
        
        if len(feedback) >= num_bad_in_row:

            bad_joint_groups = {'low_range': [], 'high_range': [], 'bad': []}
            for joint_group, _ in self.exercise_params['exercise_info'][self.curr_exercise]['comparison_joints']:
                for message in ['low_range', 'high_range', 'bad']:
                    to_check = []
                    for ii in range(-num_bad_in_row, 0):
                        to_check.append(feedback[ii]['correction'][joint_group])
                    count = [1 for tmp in to_check if message in tmp]
                    count = np.sum(count)
                    if count == num_bad_in_row:
                        bad_joint_groups[message].append(joint_group) 
            
            for message, bad_joints in bad_joint_groups.items():
                if len(bad_joints) > 0:
                    left_count = np.sum([1 for j in bad_joints if 'left' in j ])
                    right_count = np.sum([1 for j in bad_joints if 'right' in j ])
                    
                    if right_count == 0:
                        c.append('{} left side'.format(message))
                    elif left_count == 0:
                        c.append('{} right side'.format(message))
                    else:
                        c.append('{} both sides'.format(message))

            if len(c) > 0:
                for message in ['low_range', 'high_range', 'bad']:
                    if len(self.set_data_dict[self.curr_set]['eval_case_log']) > 1:
                        for e in self.set_data_dict[self.curr_set]['eval_case_log'][-1]:
                            if message in e:
                                c = []

        return c
    
    def get_correction_eval_cases(self, feedback):
        c = []

        if np.min(feedback[-1]['evaluation']) < 0:
            return c

        num_bad_in_row = 4 - self.verbal_cadence
        
        if len(feedback) >= num_bad_in_row + 1:

            bad_joint_groups = {'low_range': [], 'high_range': [], 'bad': []}
            for joint_group, _ in self.exercise_params['exercise_info'][self.curr_exercise]['comparison_joints']:
                for message in ['low_range', 'high_range', 'bad']:
                    to_check = []
                    for ii in range(-num_bad_in_row-1, -1):
                        to_check.append(feedback[ii]['correction'][joint_group])
                    count = [1 for tmp in to_check if message in tmp]
                    count = np.sum(count)
                    if count == num_bad_in_row:
                        bad_joint_groups[message].append(joint_group) 
            
            for message, bad_joints in bad_joint_groups.items():
                if len(bad_joints) > 0:
                    left_count = np.sum([1 for j in bad_joints if 'left' in j ])
                    right_count = np.sum([1 for j in bad_joints if 'right' in j ])
                    
                    if right_count == 0:
                        c.append('corrected {} left side'.format(message))
                    elif left_count == 0:
                        c.append('corrected {} right side'.format(message))
                    else:
                        c.append('corrected {} both sides'.format(message))

        return c
    
    def get_good_eval_cases(self, feedback):
        c = []

        num_good_in_row = 5 - self.verbal_cadence

        if len(feedback) >= num_good_in_row:
            for ii in range(-num_good_in_row, 0):
                if np.min(feedback[ii]['evaluation']) < 0:
                    return c
            
            c.append('good form')
            #last positive message
            
            if len(self.set_data_dict[self.curr_set]['eval_case_log']) >= num_good_in_row:
                for ii in range(-num_good_in_row, 0):
                    if 'good form' in self.set_data_dict[self.curr_set]['eval_case_log'][ii]:
                        c = []

        return c
    
    def get_bad_speed_cases(self, feedback):
        c = []

        num_bad_in_row = 5 - self.verbal_cadence
        if len(feedback) >= num_bad_in_row:

            fast_count = 0
            slow_count = 0
            for ii in range(-num_bad_in_row, 0):
                if feedback[ii]['speed'] == 'fast':
                    fast_count += 1
                if feedback[ii]['speed'] == 'slow':
                    slow_count += 1
            
            if fast_count == num_bad_in_row:
                c.append('fast')
            
            if slow_count == num_bad_in_row:
                c.append('slow')

            if len(self.set_data_dict[self.curr_set]['speed_case_log']) >= num_bad_in_row:
                for message in ['fast', 'slow']:
                    if message in self.set_data_dict[self.curr_set]['speed_case_log'][-1]:
                        c = []

        return c
    
    def get_correction_speed_cases(self, feedback):
        c = []

        if not feedback[-1]['speed'] == 'good':
            return c

        num_bad_in_row = 5 - self.verbal_cadence

        if len(feedback) >= num_bad_in_row + 1:

            fast_count = 0
            slow_count = 0
            for ii in range(-num_bad_in_row-1, -1):
                if feedback[ii]['speed'] == 'fast':
                    fast_count += 1
                if feedback[ii]['speed'] == 'slow':
                    slow_count += 1
            
            if fast_count == num_bad_in_row:
                c.append('corrected fast')
            
            if slow_count == num_bad_in_row:
                c.append('corrected slow')
        
        return c
    
    def get_good_speed_cases(self, feedback):
        c = []

        num_good_in_row = 5 - self.verbal_cadence

        if len(feedback) >= num_good_in_row:
            for ii in range(-num_good_in_row, 0):
                if not feedback[ii]['speed'] == 'good':
                    return c
            
            c.append('good speed')

            #last positive message
            if len(self.set_data_dict[self.curr_set]['speed_case_log']) >= num_good_in_row:
                for ii in range(num_good_in_row):
                    if 'good speed' in self.set_data_dict[self.curr_set]['speed_case_log'][-ii]:
                        c = []

        return c
    
    def find_eval_case(self, feedback):
        c = []

        if self.verbal_cadence == 0:
            #no verbal feedback
            return c

        c.extend(self.get_bad_eval_cases(feedback))
        c.extend(self.get_correction_eval_cases(feedback))
        c.extend(self.get_good_eval_cases(feedback))
       
        return c
    
    def find_speed_case(self, feedback):
        c = []

        if self.verbal_cadence == 0:
            #no verbal feedback
            return c

        c.extend(self.get_bad_speed_cases(feedback))
        c.extend(self.get_correction_speed_cases(feedback))
        c.extend(self.get_good_speed_cases(feedback))
        
        return c
    
    def get_pepper_message(self, c):
        styles = ['very firm', 'firm', 'neutral', 'encouraging', 'very encouraging']
        m = []
        message_to_case = []
        for ci in c:

            key1 = self.curr_exercise.replace('_', ' ')
            # key2 = 'low'
            curr_hrr = self.set_data_dict[self.curr_set]['hrr'][-1]
            curr_heart_rate = self.set_data_dict[self.curr_set]['heart_rates'][-1]
            if curr_hrr < 0.2:
                key2 = 'low'
            elif curr_hrr < 0.4:
                key2 = 'moderate'
            else:
                key2 = 'high'
            self.logger.info('Heart Rate is {}, HRR is {}, Fatigue is {}'.format(curr_heart_rate, curr_hrr, key2))

            key3 = ci
            key4 = styles[self.robot_style]
            m_to_add = self.pepper_messages[key1][key2][key3][key4]

            m.extend(m_to_add)
            message_to_case.extend([ci]*len(m_to_add))

        if len(m) > 0:
            #Pick the option that has been chosen the least
            counts = []
            for option in m:
                if option in self.set_data_dict[self.curr_set]['message_log']:
                    counts.append(self.self.set_data_dict[self.curr_set]['message_log'].count(option))
                else:
                    counts.append(0)
            
            #Get the minimum count
            ind = np.argmin(counts)
            
            return message_to_case[ind], m[ind]

        return -1, ''
    
    def react_nonverbal(self, value):

        if value == 'neutral':
            print("neutral non verbal reaction")
            a = -0.1
            b = 0.1
            start_position = (self.neutral_posture + (b-a) * np.random.random_sample((5,)) + a).tolist()
            end_position = (self.neutral_posture + (b-a) * np.random.random_sample((5,)) + a).tolist()
            ##TODO: update with pepper expression
            # self.send_body(start_position, end_position, 2)

        elif value == 'positive':
            print("positive nonverbal reaction")
            end_arm = [0, -0.8]
            start_position = [end_arm[0], end_arm[1], end_arm[0], end_arm[1], self.exercise_params['nonverbal_react']['positive']['torso'][self.robot_style]]
            ##TODO: update with pepper expression
            # self.send_body(start_position, self.neutral_posture, 4)

        elif value == 'negative':
            print("negative nonverbal reaction")
            start_position = self.neutral_posture
            start_position[-1] = self.exercise_params['nonverbal_react']['negative']['torso'][self.robot_style]
            ##TODO: update with pepper expression
            # self.send_body(start_position, self.neutral_posture, 4)

    def nonverbal_case(self, feedback, c):
        if c == '':
            if np.min(feedback[-1]['evaluation']) >= 0:
                if self.nonverbal_cadence == 3:
                    self.react_nonverbal('positive')
                elif self.nonverbal_cadence == 2 and np.random.random_sample() < 0.5:
                    self.react_nonverbal('positive')
                elif self.nonverbal_cadence == 1 and np.random.random_sample() < 0.25:
                    self.react_nonverbal('positive')
                else:
                    self.react_nonverbal('neutral')
            else:
                if self.nonverbal_cadence == 3:
                    self.react_nonverbal('negative')
                elif self.nonverbal_cadence == 2 and np.random.random_sample() < 0.5:
                    self.react_nonverbal('negative')
                elif self.nonverbal_cadence == 1 and np.random.random_sample() < 0.25:
                    self.react_nonverbal('negative')
                else:
                    self.react_nonverbal('neutral')        
        else:
            if 'good' in c or 'corrected' in c:
                self.react_nonverbal('positive')
            else:
                self.react_nonverbal('negative')
    
    def nonverbal_case(self, feedback, c):
        if c == '':
            if np.min(feedback[-1]['evaluation']) >= 0:
                if self.nonverbal_cadence == 3:
                    self.react_nonverbal('positive')
                elif self.nonverbal_cadence == 2 and np.random.random_sample() < 0.5:
                    self.react_nonverbal('positive')
                elif self.nonverbal_cadence == 1 and np.random.random_sample() < 0.25:
                    self.react_nonverbal('positive')
                else:
                    self.react_nonverbal('neutral')
            else:
                if self.nonverbal_cadence == 3:
                    self.react_nonverbal('negative')
                elif self.nonverbal_cadence == 2 and np.random.random_sample() < 0.5:
                    self.react_nonverbal('negative')
                elif self.nonverbal_cadence == 1 and np.random.random_sample() < 0.25:
                    self.react_nonverbal('negative')
                else:
                    self.react_nonverbal('neutral')        
        else:
            if 'good' in c or 'corrected' in c:
                self.react_nonverbal('positive')
            else:
                self.react_nonverbal('negative')
    
    def react(self, feedback):
        #We have feedback/evaluation from Rep i
        if np.min(feedback[-1]['evaluation']) >= 0:
            reward_i = 1
        else:
            reward_i = 0
               
        #Let's get context i
        curr_hrr = self.set_data_dict[self.curr_set]['hrr'][-1]
        if curr_hrr < 0.2:
            context_i = 0
        elif curr_hrr < 0.4:
            context_i = 1
        else:
            context_i = 2
        
        self.logger.info('Current Context {}'.format(context_i))

        # #Now let's decide which style to react to rep_i with given the context_i and reward_i
        if self.adaptive:
            try:
                self.process.stdin.write(f"{context_i},{-1},{-1}\n")
                self.process.stdin.flush()
            except BrokenPipeError:
                print('Broken Pipe Error')

            result = self.process.stdout.readline().strip()
            
            #Save the previous action choice and update the robot style temporarily
            previous_action_choice = self.robot_style
            print("CASE 1: ROBOT STYLE", self.adaptive)
            action_choice = int(result)
            self.update_robot_style(action_choice)
        else:
            print("CASE 2: ROBOT STYLE", self.adaptive)
            action_choice = self.robot_style
        
        print('ACTION CHOICE', action_choice)
        # self.logger.info('Action Choice from the model', action_choice) ##TODO: fix this logger

        #Let's see if we are actually going to react though
        eval_case = self.find_eval_case(feedback)
        self.set_data_dict[self.curr_set]['eval_case_log'].append(eval_case)

        speed_case = self.find_speed_case(feedback)
        self.set_data_dict[self.curr_set]['speed_case_log'].append(speed_case)

        # Get message for each case
        eval_chosen_case, eval_message = self.get_pepper_message(eval_case)
        speed_chosen_case, speed_message = self.get_pepper_message(speed_case)

        self.logger.info('Evaluation case {} with chosen {} and message - {}'.format(eval_case, eval_chosen_case, eval_message))
        self.logger.info('Speed case {} with chosen {} and message - {}'.format(speed_case, speed_chosen_case, speed_message))
        
        #If both messages available, choose the eval message
        returned = False
        if len(eval_message) > 0:
            chosen_case = eval_chosen_case
            returned = self.message(eval_message, priority=1)
        elif len(speed_message) > 0:
            chosen_case = speed_chosen_case
            returned = self.message(speed_message, priority=1)
        
        #If feedback case, want to match the reaction, otherwise nonverbal react based on cadence
        if speed_message == '' and eval_message == '':
            self.nonverbal_case(feedback, '')
        else:
            self.nonverbal_case(feedback, chosen_case)

        #If we reacted, then we leave the action choice as is
        if returned:
            pass
            # self.logger.info('Robot said the message')
        elif self.adaptive:
            #Go back to the previous action chocie, since we did not use the new one
            self.logger.info('Robot did not say the message, going back to previous action choice of {}'.format(previous_action_choice))
            self.update_robot_style(previous_action_choice)
        
        action_i = 0 if self.robot_style == 1 else 1 #0 for firm, 1 for encouraging
        self.set_data_dict[self.curr_set]['contexts'].append(context_i)
        self.set_data_dict[self.curr_set]['actions'].append(action_i)
        self.set_data_dict[self.curr_set]['rewards'].append(reward_i)

        #We need to train as well
        if self.adaptive:
            #We train on context_i-1, action_i-1, reward_i
            if len(self.contexts) == 1:
                training_context = 0
                training_action = 1
            else:
                training_context = self.contexts[-2]
                training_action = self.actions[-2]
            training_reward = self.rewards[-1]
            self.logger.info('Training on Context {}, Action {}, Reward {}'.format(training_context, training_action, training_reward))
            try:
                self.process.stdin.write(f"{training_context},{training_action},{training_reward}\n")
                self.process.stdin.flush()
            except BrokenPipeError:
                print('Broken Pipe Error')

            result = self.process.stdout.readline().strip()
            if result:
                print('Trained model with Context {}, Action {}, Reward {}'.format(training_context, training_action, training_reward))

    def evaluate_rep(self, prev_rep_data_idx, rep_duration):
        
        corrections = {}
        eval_list = []

        for joint_group, angles_to_include in self.exercise_params['exercise_info'][self.curr_exercise]['comparison_joints']:
            indices = self.exercise_params['angle_order'][joint_group]
            indices_to_include = []
            for ii in range(3):
                if ii in angles_to_include:
                    indices_to_include.append(indices[ii])
                
            distances, labels = self.calc_dist(prev_rep_data_idx, indices_to_include, ['good', 'low range', 'high range'])

            good_experts = distances[0:3]
            good_distance = np.min(good_experts)

            closest_expert = np.argmin(distances)
            best_distance = distances[closest_expert]
            expert_label = labels[closest_expert]

            if (good_distance > best_distance and good_distance < self.exercise_params['exercise_info'][self.curr_exercise]['threshold1']):
                eval_list.append(1)
                correction = 'good'
            
            elif (best_distance < self.exercise_params['exercise_info'][self.curr_exercise]['threshold1']):
                if expert_label == 'good':
                    eval_list.append(1)
                else:
                    eval_list.append(-1)
                correction = expert_label
            
            elif (good_distance > best_distance and good_distance < self.exercise_params['exercise_info'][self.curr_exercise]['threshold2']):
                eval_list.append(0)
                correction = 'ok'
            
            elif (best_distance < self.exercise_params['exercise_info'][self.curr_exercise]['threshold2']):
                if expert_label == 'good':
                    correction = 'ok'
                    eval_list.append(0)
                else:
                    correction = 'bad'
                    eval_list.append(-1)
            
            else:
                correction = 'bad'
                eval_list.append(-1)
            
            correction += ' {}'.format(joint_group)
            corrections[joint_group] = correction
        
        if rep_duration < self.experts[self.curr_exercise]['average duration'] - 3:
            speed = 'fast'
        elif rep_duration > self.experts[self.curr_exercise]['average duration'] + 3 and rep_duration < 7:
            speed = 'slow'
        else:
            speed = 'good'
        
        self.logger.info('Actual Duration {}, Average Expert Duration {}'.format(np.round(rep_duration, 1), np.round(self.experts[self.curr_exercise]['average duration'], 1)))

        feedback = {'speed': speed, 'correction': corrections, 'evaluation': eval_list}
        self.set_data_dict[self.curr_set]['feedback'].append(feedback)
        self.set_data_dict[self.curr_set]['performance'].append(feedback['evaluation'])
        
        self.logger.info('Rep {}: Feedback {}'.format(len(self.set_data_dict[self.curr_set]['feedback']), feedback))
        self.react(self.set_data_dict[self.curr_set]['feedback'])
        
        return feedback

    
    def pose_callback(self, angle_message):
        if self.curr_set == '':
            return
        
        #grab data from message
        next_joint_angles = list(angle_message.data)
        next_timestamp = rospy.Time.now().to_sec()

        #Save heart rate and hrr
        if(len(self.set_data_dict[self.curr_set]['all_heart_rates'])) > 0:
            curr_heart_rate = self.set_data_dict[self.curr_set]['all_heart_rates'][-1]
            #push back current heart rate
            self.set_data_dict[self.curr_set]['heart_rates'].append(curr_heart_rate)
            #calculate the hrr
            hrr = (curr_heart_rate - self.resting_hr) / float(self.max_hr - self.resting_hr)
            self.set_data_dict[self.curr_set]['hrr'].append(hrr)

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
        exercise_min = self.exercise_params['exercise_info'][self.curr_exercise]['current_angles_min']
        exercise_grad = self.exercise_params['exercise_info'][self.curr_exercise]['max_grad']

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
                rep_duration = curr_timestamp - prev_peak_timestamp
                is_new_peak = True if (rep_duration > timestamp_threshold) else False
            else:
                rep_duration = -1
                is_new_peak = True
            #combine conditional statements
            if is_peak and is_arm_raised and is_min_angle_reached and is_new_peak:
                print("NEW PEAK DETECTED", rep_duration)
                self.set_data_dict[self.curr_set]['peaks'].append(curr_timestamp)
                #evaluate rep
                if(len(self.set_data_dict[self.curr_set]['peaks']) > 1):
                    #grab timestep of previous rep
                    prev_rep_timestep = self.set_data_dict[self.curr_set]['peaks'][-2]
                    #grab data for prev rep
                    prev_rep_data_idx = next(i for i, data in enumerate(self.set_data_dict[self.curr_set]['joint_angles']) 
                                            if data[-1] == prev_rep_timestep)
                    # prev_rep_data = self.set_data_dict[self.curr_set]['joint_angles'][prev_rep_data_idx]
                    # evaluate based on current and previous rep
                    self.evaluate_rep(prev_rep_data_idx, curr_timestamp - prev_rep_timestep)
                    ##TODO: add a timer on the evaluation
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
        # if (len(self.message_time_stamps)) > 0:
        #     last_message_time = self.message_time_stamps[-1]
        #     if (datetime.now(timezone('EST')) - last_message_time).total_seconds() < 3 and priority < 2:
        #         #Skip message
        #         self.logger.info('Skipping {}'.format(m))
        #         return False
                
        self.logger.info('Robot says: {}'.format(m))
        length_estimate = np.round(self.slope*syllables.estimate(m) + self.intercept)

        if not self.replay:
            self.sound_pub.publish(m)

        # self.message_log.append(m)
        # self.message_time_stamps.append(datetime.now(timezone('EST')) + timedelta(seconds=length_estimate) )
        return True    

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
        #add feedback lists to dictionary
        self.set_data_dict[self.curr_set]['feedback'] = []
        #add performance lists to dictionary
        self.set_data_dict[self.curr_set]['performance'] = []
        #add heart rate to dictionary
        self.set_data_dict[self.curr_set]['all_heart_rates'] = []
        #add exercise heart rate to dictionary
        self.set_data_dict[self.curr_set]['heart_rates'] = []
        #add hrr to dictionary
        self.set_data_dict[self.curr_set]['hrr'] = []
        #add eval case log to dictionary
        self.set_data_dict[self.curr_set]['eval_case_log'] = []
        #add speed case log to dictionary
        self.set_data_dict[self.curr_set]['speed_case_log'] = []
        #add message log to dictionary
        self.set_data_dict[self.curr_set]['message_log'] = []
        #specify angles to compare for pose callback
        self.set_data_dict[self.curr_set]['angle_compare_idx'] = []
        #add contexts to data dictionary
        self.set_data_dict[self.curr_set]['contexts'] = []
        #add actions to data dictionary
        self.set_data_dict[self.curr_set]['actions'] = []
        #add rewards to data dictionary
        self.set_data_dict[self.curr_set]['rewards'] = []
        for group, ind in self.exercise_params['exercise_info'][exercise_name]['segmenting_joints']:
            self.set_data_dict[self.curr_set]['angle_compare_idx'].append(
                self.exercise_params['angle_order'][group][ind]
            )
        #Raise arm all the way up
        # self.move_right_arm('up', 'halfway') ##TODO: replace with pepper command

        #Add starting set to log
        self.logger.info('=====================================')
        self.logger.info('STARTING SET {} OF {} - {}'.format(set_num, tot_sets, self.curr_exercise))
        self.logger.info('=====================================')

        #Robot says starting set and smile
        robot_message = "Get ready to start set %s of %s. You will be doing %s." % (set_num, tot_sets, exercise_name.replace("_", " " ))
        self.message(robot_message)
        # self.change_expression('smile', self.start_set_smile, 4) ##TODO: change to smile based command for pepper robot


##MAIN FUNCTION FOR STANDALONE EXERCISE CONTROLLER
if __name__ == '__main__':
    
    PARTICIPANT_ID = '0'
    ROBOT_STYLE = 5 #1 is firm, 3 is encouraging, 5 is adaptive
    RESTING_HR = 97
    MAX_HR = 220-26
    LOG_FILENAME = "test.pickle"

    rospy.init_node('exercise_manager_node')

    controller = ExerciseManager(False, LOG_FILENAME, ROBOT_STYLE, RESTING_HR, MAX_HR, PARTICIPANT_ID)
    rospy.sleep(2.0)

    controller.start_new_set(exercise_name='bicep_curls',set_num=1, tot_sets=1)

    rospy.spin()

