#!/usr/bin/env python3
from __future__ import print_function
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import cv2
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../pytorch'))
from nav_cloning_pytorch import *
from skimage.transform import resize
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from std_srvs.srv import SetBool, SetBoolResponse
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
import csv
import copy
import time
import tf
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import PoseStamped

class nav_cloning_node:
    def __init__(self):
        rospy.init_node('nav_cloning_node', anonymous=True)
        self.mode = rospy.get_param("/nav_cloning_node/mode", "use_dl_output")
        self.action_num = 1
        self.dl = deep_learning(n_action = self.action_num)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.nav_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.gazebo_pos_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_gazebo_pos, queue_size = 2)
        self.min_distance = 0.0
        self.episode = 0
        self.path_point = []
        self.redpoint = []
        self.first_move = True
        self.cv_image = np.zeros((480,640,3), np.uint8)
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.path = roslib.packages.get_pkg_dir('nav_cloning')+'/data/path.csv'
        self.file_path = roslib.packages.get_pkg_dir('nav_cloning')+'/data/result_image/selected_training/thesis/1/threshold_0.29/redpoint1.csv'
        self.load_path = roslib.packages.get_pkg_dir('nav_cloning')+'/data/model_selected_training/thesis/1/'
        print(self.load_path)
        with open(self.path, 'r') as f:
            for row in csv.reader(f):
                self.path_point.append(row)
            print(self.path)

        with open(self.file_path, 'r') as f:
            for row in csv.reader(f):
                self.redpoint.append(row)
        if not self.redpoint:
            os.system('killall roslaunch')
            sys.exit()

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    
    def check_traceable(self):
        if self.min_distance <= 0.05:
            traceable = True
        else:
            traceable = False
        return traceable
    
    def callback_gazebo_pos(self, data):
        self.gazebo_pos_x = data.pose[2].position.x
        self.gazebo_pos_y = data.pose[2].position.y

    def check_distance(self):
        distance_list = []
        for pose in self.path:
            path_x = pose[0]
            path_y = pose[1]
            distance = np.sqrt(abs((self.gazebo_pos_x - path_x)**2 + (self.gazebo_pos_y - path_y)**2))
            distance_list.append(distance)
        self.min_distance = min(distance_list)
        print("ditance: " + str(self.min_distance))
    
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

    def robot_move(self, count):
        if self.first_move:
            with open(self.file_path, 'r') as f:
                for row in csv.reader(f):
                    self.redpoint.append(row)
        x, y, z, w = self.redpoint[count].split(',')
        print(x)
        r = rospy.Rate(10)
        rospy.wait_for_service('/gazebo/set_model_state')
        state = ModelState()
        state.model_name = 'mobile_base'
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.orientation.x = 0
        state.pose.orientation.y = 0
        state.pose.orientation.z = z
        state.pose.orientation.w = w
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state )
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        r.sleep() #need adjust
        r.sleep() #need adjust
        r.sleep() #need adjust

    def loop(self):
        if self.cv_image.size != 640 * 480 * 3:
            return

        img = resize(self.cv_image, (48, 64), mode='constant')
        self.check_distance() 
        traceable = self.check_traceable()

        if self.episode > 5:
            collision_flag = self.collision()
        
        if traceable or collision_flag:
            self.robot_move()
        target_action = self.dl.act(img)
        print("episode:" +str(self.episode))
        self.episode += 1

        if self.episode <= 5:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
        else:
            self.vel.linear.x = 0.2
            self.vel.angular.z = target_action
        self.nav_pub.publish(self.vel)

        temp = copy.deepcopy(img)
        cv2.imshow("Resized Image", temp)
        cv2.waitKey(1)

    
if __name__ == '__main__':
    rg = nav_cloning_node()
    DURATION = 0.2
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()