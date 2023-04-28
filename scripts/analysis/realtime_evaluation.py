#!/usr/bin/env python3
from __future__ import print_function

import roslib
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../pytorch")
roslib.load_manifest('nav_cloning')
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_cloning_pytorch import *
from skimage.transform import resize
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import SetBool, SetBoolResponse
import time
import copy
import tf
from nav_msgs.msg import Odometry
import numpy as np
import csv
import pathlib

class nav_cloning_node:
    def __init__(self):
        rospy.init_node('nav_cloning_node', anonymous=True)
        self.mode = rospy.get_param("/nav_cloning_node/mode", "selected_training")
        self.action_num = 1
        self.dl = deep_learning(n_action = self.action_num)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.vel_sub = rospy.Subscriber("/nav_vel", Twist, self.callback_vel)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_pose)
        self.gazebo_pose = rospy.Subscriber("/tracker", Odometry, self.callback_gazebo_pose)
        self.action = 0.0
        self.episode = 0
        self.vel = Twist()
        self.cv_image = np.zeros((480,640,3), np.uint8)
        #------------------------------
        arg = int(sys.argv[1])
        if arg % 2 == 0:
            if arg % 4 == 0:
                self.model_num = 4
            else:
                self.model_num = 2
        else:
            self.model_num = arg - (int(arg / 4) * 4)
        self.dirnum = sys.argv[2]
        #------------------------------
        self.red_threshold = 0.2 #0.31
        self.yellow_threshold = 0.1 #0.25
        self.path = roslib.packages.get_pkg_dir('nav_cloning')+'/data/result_'+str(self.mode)+'/'
        self.save_path = roslib.packages.get_pkg_dir('nav_cloning')+'/data/model_'+str(self.mode)+'/model1.net'
        self.load_path = roslib.packages.get_pkg_dir('nav_cloning')+'/data/model_'+str(self.mode)+'/thesis/'+str(self.dirnum)+'/model'
        self.image = cv2.imread(roslib.packages.get_pkg_dir('nav_cloning')+'/maps/willowgarage-refined.pgm')
        self.image_resize = cv2.resize(self.image, (600, 600))
        self.img_path = roslib.packages.get_pkg_dir('nav_cloning')+'/data/result_image/'+str(self.mode)+'/thesis/'+str(self.dirnum)+'/threshold_'+str(self.red_threshold)+'/'
        self.srv = rospy.Service('/save_img', Trigger, self.callback_srv)
        self.start_time_s = rospy.get_time()
        self.pos_x, self.pos_y = 0.0, 0.0
        self.orientation_z, self.orientation_w = 0.0, 0.0
        self.gazebo_pos_x, self.gazebo_pos_y = 0.0, 0.0
        self.gazebo_orientation_z, self.gazebo_orientation_w = 0.0, 0.0
        self.old_pos_x, self.old_pos_y = self.pos_x, self.pos_y
        self.red_count, self.yellow_count = 0, 0
        self.flag = False
        self.dl.load(self.load_path + str(self.model_num) + '.pt')
        print(self.load_path + str(self.model_num) + '.pt')
        # os.makedirs(self.img_path, exist_ok=True)
        # touch_file = pathlib.Path(self.img_path + 'redpoint'+str(self.model_num)+'.csv')
        # touch_file.touch()

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    
    def callback_pose(self, data):
        self.pos_x = data.pose.pose.position.x
        self.pos_y = data.pose.pose.position.y
        self.orientation_z = data.pose.pose.orientation.z
        self.orientation_w = data.pose.pose.orientation.w
        self.vis_x = 205 + int(self.pos_x * 10.7)
        self.vis_y = 325 + int(self.pos_y * 13.4) * (-1)
        self.old_vis_x = 205 + int(self.old_pos_x * 10.7)
        self.old_vis_y = 325 + int(self.old_pos_y * 13.4) * (-1)
        if self.old_vis_x != 205:
            self.flag = True
    
    def callback_vel(self, data):
        self.vel = data
        self.action = self.vel.angular.z
    
    def callback_gazebo_pose(self, data):
        self.gazebo_pos_x = data.pose.pose.position.x
        self.gazebo_pos_y = data.pose.pose.position.y
        self.gazebo_orientation_z = data.pose.pose.orientation.z
        self.gazebo_orientation_w = data.pose.pose.orientation.w
    
    def draw_red_circle(self, vis_x, vis_y):
        cv2.circle(self.image_resize, (vis_x, vis_y), 3, (0, 0, 255), thickness = 3)

    def draw_yellow_circle(self, vis_x, vis_y):
        cv2.circle(self.image_resize, (vis_x, vis_y), 3, (0, 255, 255), thickness = 3)

    def draw_line(self, vis_x, vis_y, old_vis_x, old_vis_y):
        cv2.line(self.image_resize, (vis_x, vis_y), (old_vis_x, old_vis_y), (0, 200, 0), thickness=2)

    def draw_translucent_rect(self, img, x, y, w, h):
        sub_img = img[y:y+h, x:x+w]
        black_rect = np.zeros(sub_img.shape, dtype=np.uint8)
        rect = cv2.addWeighted(sub_img, 0.2, black_rect, 0.2, 1.0)
        img[y:y+h, x:x+w] = rect

    def draw_text_with_box(self, img, text, org, padding, face, height, color, thickness=1, lineType=cv2.LINE_8):
        scale = cv2.getFontScaleFromHeight(face, height, thickness)
        size, baseline = cv2.getTextSize(text, face, scale, thickness)
        rect_x = org[0] - padding
        rect_y = org[1] - height - padding
        rect_w = size[0] + (padding*2)
        rect_h = size[1] + baseline + (padding*2)
        rect_x = max(rect_x, 0)
        rect_y = max(rect_y, 0)
        max_w = img.shape[1] - rect_x
        max_h = img.shape[0] - rect_y
        rect_w = min(rect_w, max_w)
        rect_h = min(rect_h, max_h)
        self.draw_translucent_rect(img, rect_x, rect_y, rect_w, rect_h)
        cv2.putText(img, text, org, face, scale, color, thickness)


    def callback_srv(self, data):
        resp = TriggerResponse()
        resp.success = True
        resp.message = "save image"
        cv2.imwrite(self.img_path + 'model' + str(self.model_num) + '.png', self.crop_img)
        return resp

    def loop(self):
        if self.cv_image.size != 640 * 480 * 3:
            return
        img = resize(self.cv_image, (48, 64), mode='constant')

        # if self.episode == 1750:
        #     cv2.imwrite(self.img_path + 'model' + str(self.model_num) + '.png', self.crop_img)
        #     sys.exit()

        target_action = self.dl.act(img)
        angle_error = abs(self.action - target_action)
        print(f'{self.episode:04}' + ", test, angle_error:" + f'{angle_error:.015f}' + ", red point sum:" + str(self.red_count) + ", yellow point sum:" + str(self.yellow_count))
        self.episode += 1
        if self.flag:
            if angle_error > self.red_threshold:
                self.draw_red_circle(self.vis_x, self.vis_y)
                # self.redposition = [str(self.gazebo_pos_x), str(self.gazebo_pos_y), str(self.gazebo_orientation_z), str(self.gazebo_orientation_w)]
                # with open(self.img_path + 'redpoint'+str(self.model_num)+'.csv', 'a') as f:
                #     writer = csv.writer(f, lineterminator='\n')
                #     writer.writerow(self.redposition)
                self.red_count += 1
            if angle_error > self.yellow_threshold and angle_error < self.red_threshold:
                self.draw_yellow_circle(self.vis_x, self.vis_y)
                self.yellow_count += 1
            self.draw_line(self.vis_x, self.vis_y, self.old_vis_x, self.old_vis_y)
        self.crop_img = self.image_resize.copy()
        self.crop_img = self.crop_img[301:600, 0:400]
        self.old_pos_x = self.pos_x
        self.old_pos_y = self.pos_y
        self.draw_text_with_box(self.crop_img, 'red point sum:' + str(self.red_count), (5, 25), 0, cv2.FONT_HERSHEY_COMPLEX, 12, (0,0,255), 1, cv2.LINE_AA)
        self.draw_text_with_box(self.crop_img, 'yellow point sum:' + str(self.yellow_count), (180, 25), 0, cv2.FONT_HERSHEY_COMPLEX, 12, (0,255,255), 1, cv2.LINE_AA)
        self.draw_text_with_box(self.crop_img, 'red threshold:' + str(self.red_threshold), (5, 50), 0, cv2.FONT_HERSHEY_COMPLEX, 12, (255,255,255), 1, cv2.LINE_AA)
        self.draw_text_with_box(self.crop_img, 'yellow threshold:' + str(self.yellow_threshold), (180, 50), 0, cv2.FONT_HERSHEY_COMPLEX, 12, (255,255,255), 1, cv2.LINE_AA)
        cv2.imshow("realtime_evaluation", self.crop_img)
        temp = copy.deepcopy(img)
        cv2.waitKey(1)

        if self.old_pos_x == self.pos_x:
            self.flag = False
            
if __name__ == '__main__':
    rg = nav_cloning_node()
    DURATION = 0.1
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()