#!/usr/bin/env python3

from __future__ import print_function
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import SetBool, SetBoolResponse
from matplotlib import pyplot as plt 
import numpy as np

class realtime_evaluation:
    def __init__(self):
        rospy.init_node('realtime_evaluation', anonymous=True)
        self.nav_sub = rospy.Subscriber("/nav_vel", Twist, self.callback_vel)
        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.callback_cmd_vel)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_pose)
        self.image = cv2.imread(roslib.packages.get_pkg_dir('nav_cloning')+'/maps/willowgarage-refined.pgm')
        # self.image = cv2.imread(roslib.packages.get_pkg_dir('nav_cloning')+'/maps/map.png')
        self.image_resize = cv2.resize(self.image, (600, 600))
        self.path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/result_image/model10.png'
        # self.path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/result_image/'
        self.srv = rospy.Service('/save_img', Trigger, self.callback_srv)
        # self.srv = rospy.Service('/save_img', SetBool, self.callback_srv)
        self.action = 0.0
        self.vel = Twist()
        self.cmd_action = 0.0
        self.cmd_vel = Twist()
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.old_pos_x = self.pos_x
        self.old_pos_y = self.pos_y
        self.diff = 0.0
        self.count = 0
        self.flag = False

    def callback_pose(self, data):
        self.pos_x = data.pose.pose.position.x
        self.pos_y = data.pose.pose.position.y
        self.vis_x = 205 + int(self.pos_x * 10.7)
        self.vis_y = 325 + int(self.pos_y * 13.4) * (-1)
        self.old_vis_x = 205 + int(self.old_pos_x * 10.7)
        self.old_vis_y = 325 + int(self.old_pos_y * 13.4) * (-1)
        if self.old_vis_x != 205:
            self.flag = True
    
    def callback_vel(self, data):
        self.vel = data
        self.action = self.vel.angular.z

    def callback_cmd_vel(self, data):
        self.cmd_vel = data
        self.cmd_action = self.cmd_vel.angular.z

    def draw_circle(self, vis_x, vis_y):
        cv2.circle(self.image_resize, (vis_x, vis_y), 3, (0, 0, 255), thickness = 3)

    def draw_line(self, vis_x, vis_y, old_vis_x, old_vis_y):
        cv2.line(self.image_resize, (vis_x, vis_y), (old_vis_x, old_vis_y), (0, 200, 0), thickness=2)
    
    def callback_srv(self, data):
        resp = TriggerResponse()
        resp.success = True
        resp.message = "save image"
        cv2.imwrite(self.path, self.crop_img)
        return resp

    def loop(self): 
        if self.flag:
            self.diff = abs(self.action - self.cmd_action)
            print(self.diff)
            if self.diff > 0.2:
                self.draw_circle(self.vis_x, self.vis_y)
                self.count += 1
            self.draw_line(self.vis_x, self.vis_y, self.old_vis_x, self.old_vis_y)
            print("red point:" + str(self.count))
            self.crop_img = self.image_resize.copy()
            self.crop_img = self.crop_img[301:600, 0:600]
            self.old_pos_x = self.pos_x
            self.old_pos_y = self.pos_y
            cv2.putText(self.crop_img, text = 'red point sum:' + str(self.count), org = (5, 30), fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.6, color = (0,0,255))
            cv2.imshow("realtime_evaluation", self.crop_img)
            cv2.waitKey(1)

if __name__ == '__main__':
    rg = realtime_evaluation()
    DURATION = 0.2
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()