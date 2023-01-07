#!/usr/bin/env python3
from __future__ import print_function
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../pytorch'))
from nav_cloning_pytorch import *
from skimage.transform import resize
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
import csv
import time
import copy
import numpy as np

class nav_cloning_node:
    def __init__(self):
        rospy.init_node('nav_cloning_node', anonymous=True)
        self.mode = rospy.get_param("/nav_cloning_node/mode", "use_dl_output")
        self.num = rospy.get_param("/nav_cloning_node/num", "1")
        self.action_num = 1
        self.dl = deep_learning(n_action = self.action_num)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.image_left_sub = rospy.Subscriber("/camera_left/rgb/image_raw", Image, self.callback_left_camera)
        self.image_right_sub = rospy.Subscriber("/camera_right/rgb/image_raw", Image, self.callback_right_camera)
        self.nav_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_sub = rospy.Subscriber("/nav_vel", Twist, self.callback_vel)
        self.srv = rospy.Service('/training', SetBool, self.callback_dl_training)
        self.min_distance = 0.0
        self.episode = 0
        self.vel = Twist()
        self.cv_image = np.zeros((480,640,3), np.uint8)
        self.cv_left_image = np.zeros((480,640,3), np.uint8)
        self.cv_right_image = np.zeros((480,640,3), np.uint8)
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.path = roslib.packages.get_pkg_dir('nav_cloning')+'/data'
        # if self.num % 2 == 0:
        #     self.dirnum = int(self.num / 2)
        #     self.csv_num = 2
        # else:
        #     self.dirnum = int(self.num - int(self.num / 2))
        #     self.csv_num = 1
        if self.num % 2 == 0:
            if self.num % 4 == 0:
                self.csv_num = 4
            else:
                self.csv_num = 2
        else:
            self.csv_num = self.num - (int(self.num / 4) * 4)
        self.dirnum = int((self.num + 4 - 1) / 4)
        self.threshold = 'threshold_0.25'
        self.traceable = 'traceable'+str(self.csv_num)+'.csv'
        self.trajectory = 'trajectory'+str(self.csv_num)+'.csv'
        self.result_path = roslib.packages.get_pkg_dir('nav_cloning')+'/data/result_image/selected_training/thesis/'+str(self.dirnum)+'/'+str(self.threshold)+'/result/'
        self.redpoint = roslib.packages.get_pkg_dir('nav_cloning')+'/data/result_image/selected_training/thesis/'+str(self.dirnum)+'/'+str(self.threshold)+'/redpoint'+str(self.csv_num)+'.csv'
        print(self.redpoint)
        self.load_path = roslib.packages.get_pkg_dir('nav_cloning')+'/data/model_selected_training/thesis/'+str(self.dirnum)+'/model'+str(self.csv_num)+'.pt'
        print(self.load_path)
        self.start_time_s = rospy.get_time()
        os.makedirs(self.result_path, exist_ok=True)
        self.gazebo_pos_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_gazebo_pos, queue_size = 2) 
        self.gazebo_pos_x = 0.0
        self.gazebo_pos_y = 0.0
        self.move_count = 1
        self.dl.load(self.load_path)
        self.path_points = []
        self.is_first = True
        self.start = False
        self.collision_list = [[],[]]
        self.redpoint_list = []
        with open(self.path + '/path.csv', 'r') as f:
            is_first = True
            for row in csv.reader(f):
                if is_first:
                    is_first = False
                    continue
                str_x, str_y = row
                x, y = float(str_x), float(str_y)
                self.path_points.append([x,y])

        with open(self.redpoint, 'r') as f:
            for row in csv.reader(f):
                self.redpoint_list.append(row)
        if not self.redpoint_list:
            os.system('killall roslaunch')
            sys.exit()

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_left_camera(self, data):
        try:
            self.cv_left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback_right_camera(self, data):
        try:
            self.cv_right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    
    def callback_vel(self, data):
        self.vel = data
        self.action = self.vel.angular.z

    def callback_gazebo_pos(self, data):
        self.gazebo_pos_x = data.pose[2].position.x
        self.gazebo_pos_y = data.pose[2].position.y

    def check_distance(self):
        distance_list = []
        for pose in self.path_points:
            path_x = pose[0]
            path_y = pose[1]
            distance = np.sqrt(abs((self.gazebo_pos_x - path_x)**2 + (self.gazebo_pos_y - path_y)**2))
            distance_list.append(distance)

        # if distance_list:
        self.min_distance = min(distance_list)
        print("distance: " + str(self.min_distance))


    def callback_dl_training(self, data):
        resp = SetBoolResponse()
        self.start = data.data
        resp.message = "Training: " + str(self.start)
        resp.success = True
        return resp

    def collision(self):
        collision_flag = False
        self.collision_list[0].append(self.gazebo_pos_x)
        self.collision_list[1].append(self.gazebo_pos_y)
        if len(self.collision_list[0]) == 10:
            average_x = sum(self.collision_list[0]) / len(self.collision_list[0])
            average_y = sum(self.collision_list[1]) / len(self.collision_list[1])
            distance = np.sqrt(abs((self.gazebo_pos_x - average_x)**2 + (self.gazebo_pos_y - average_y)**2))
            self.collision_list[0] = self.collision_list[0][1:]
            self.collision_list[1] = self.collision_list[1][1:]

            if distance < 0.1:
                collision_flag = True
                print("collision")

        return collision_flag


    def robot_move(self, move_count): #reset
        r = rospy.Rate(10)
        rospy.wait_for_service('/gazebo/set_model_state')
        state = ModelState()
        state.model_name = 'mobile_base'
        state.pose.position.x = float(self.redpoint_list[move_count][0])
        state.pose.position.y = float(self.redpoint_list[move_count][1])
        state.pose.orientation.x = 0
        state.pose.orientation.y = 0
        state.pose.orientation.z = float(self.redpoint_list[move_count][2])
        state.pose.orientation.w = float(self.redpoint_list[move_count][3])
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state )
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        r.sleep() #need adjust
        r.sleep() #need adjust
        r.sleep() #need adjust

    def first_move(self):
        if self.is_first:
            self.robot_move(self.move_count-1)
            print("robot_move_first")
            self.is_first = False

    def check_traceable(self):
        if self.min_distance <= 0.05:
            traceable = True
        else:
            traceable = False
        return traceable

    def loop(self):
        if self.cv_image.size != 640 * 480 * 3:
            return
        if self.cv_left_image.size != 640 * 480 * 3:
            return
        if self.cv_right_image.size != 640 * 480 * 3:
            return
        if self.start == False:
            return
        img = resize(self.cv_image, (48, 64), mode='constant')

        img_left = resize(self.cv_left_image, (48, 64), mode='constant')

        img_right = resize(self.cv_right_image, (48, 64), mode='constant')

        ros_time = str(rospy.Time.now())
        collision_flag = False
        self.check_distance() 
        traceable = self.check_traceable() #True or False
        if self.episode > 5:
            collision_flag = self.collision()
        
        if self.is_first:
            self.first_move()
            os.system('rosnode kill /rviz')
            return

        if self.episode > 500 and self.min_distance <= 0.3:
            print("-----------------------------------Success-----------------------------------")
            line = ["Success"]
            with open(self.result_path + self.traceable, 'a') as f:
                writer = csv.writer(f, lineterminator='\n')
                writer.writerow(line)
            self.collision_list = [[],[]]
            self.episode = 0
            self.move_count += 1
            if self.move_count > len(self.redpoint_list):
                os.system('killall roslaunch')
                sys.exit()
            self.robot_move(self.move_count-1)

        elif self.episode > 500 and self.min_distance > 0.3:
            print("-----------------------------------Failure-----------------------------------")
            line = ["Failure"]
            with open(self.result_path + self.traceable, 'a') as f:
                writer = csv.writer(f, lineterminator='\n')
                writer.writerow(line)
            self.collision_list = [[],[]]
            self.episode = 0
            self.move_count += 1
            if self.move_count > len(self.redpoint_list):
                os.system('killall roslaunch')
                sys.exit()
            self.robot_move(self.move_count-1)

        if collision_flag:
            print("-----------------------------------Failure-----------------------------------")
            line = ["Failure"]
            with open(self.result_path + self.traceable, 'a') as f:
                writer = csv.writer(f, lineterminator='\n')
                writer.writerow(line)
            self.collision_list = [[],[]]
            self.episode = 0
            self.move_count += 1
            if self.move_count > len(self.redpoint_list):
                os.system('killall roslaunch')
                sys.exit()
            self.robot_move(self.move_count-1)
                

        target_action = self.dl.act(img)
        print("episode:" +str(self.episode))
        print("move_count:" +str(self.move_count)+'/'+str(len(self.redpoint_list)))
        self.episode += 1

        if self.episode <= 5:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
        else:
            self.vel.linear.x = 0.2
            self.vel.angular.z = target_action
        line_trajectory = [str(self.episode), str(self.gazebo_pos_x), str(self.gazebo_pos_y), str(self.move_count), str(collision_flag)]
        with open(self.result_path + self.trajectory, 'a') as f:
            writer = csv.writer(f, lineterminator='\n')
            writer.writerow(line_trajectory)

        self.nav_pub.publish(self.vel)
        print("------------------"*5)

        temp = copy.deepcopy(img)
        cv2.imshow("Resized Image", temp)
        # temp = copy.deepcopy(img_left)
        # cv2.imshow("Resized Left Image", temp)
        # temp = copy.deepcopy(img_right)
        # cv2.imshow("Resized Right Image", temp)
        cv2.waitKey(1)

if __name__ == '__main__':
    rg = nav_cloning_node()
    DURATION = 0.2
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()
