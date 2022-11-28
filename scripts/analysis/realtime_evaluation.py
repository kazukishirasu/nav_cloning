#!/usr/bin/env python3
from __future__ import print_function

import roslib
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
roslib.load_manifest('nav_cloning')
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_cloning_net import *
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

class nav_cloning_node:
    def __init__(self):
        rospy.init_node('nav_cloning_node', anonymous=True)
        self.mode = rospy.get_param("/nav_cloning_node/mode", "use_dl_output")
        self.action_num = 1
        self.dl = deep_learning(n_action = self.action_num)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.vel_sub = rospy.Subscriber("/nav_vel", Twist, self.callback_vel)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_pose)
        self.nav_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.action = 0.0
        self.episode = 0
        self.vel = Twist()
        self.cv_image = np.zeros((480,640,3), np.uint8)
        self.start_time = time.strftime("%Y%m%d_%H:%M:%S")
        self.time = '/20221008_18:30:46/'
        self.model_num = 1
        self.threshold = 0.15
        self.path = roslib.packages.get_pkg_dir('nav_cloning')+'/data/result_'+str(self.mode)+'/'
        self.save_path = roslib.packages.get_pkg_dir('nav_cloning')+'/data/model_'+str(self.mode)+'/model1.net'
        self.load_path = roslib.packages.get_pkg_dir('nav_cloning')+'/data/model_'+str(self.mode + self.time)+'model'
        self.image = cv2.imread(roslib.packages.get_pkg_dir('nav_cloning')+'/maps/willowgarage-refined.pgm')
        self.image_resize = cv2.resize(self.image, (600, 600))
        self.img_path = roslib.packages.get_pkg_dir('nav_cloning')+'/data/result_image/'+str(self.mode)+'/'+str(self.time)+'/threshold_'+str(self.threshold)+'/'
        self.srv = rospy.Service('/save_img', Trigger, self.callback_srv)
        self.start_time_s = rospy.get_time()
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.old_pos_x = self.pos_x
        self.old_pos_y = self.pos_y
        self.count = 0
        self.flag = False
        # self.dl.load(self.load_path)
        self.dl.load(self.load_path + str(self.model_num) + '.pt')
        os.makedirs(self.img_path, exist_ok=True)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    
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
    
    def draw_circle(self, vis_x, vis_y):
        cv2.circle(self.image_resize, (vis_x, vis_y), 3, (0, 0, 255), thickness = 3)

    def draw_line(self, vis_x, vis_y, old_vis_x, old_vis_y):
        cv2.line(self.image_resize, (vis_x, vis_y), (old_vis_x, old_vis_y), (0, 200, 0), thickness=2)

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

        if self.episode == 10000:
            os.system('killall roslaunch')
            sys.exit()

        target_action = self.dl.act(img)
        angle_error = abs(self.action - target_action)
        # print(str(self.episode) + ", test, angular:" + str(target_action) + ", angle_error:" + str(angle_error) + ", red point sum:" + str(self.count))
        print(f'{self.episode:04}' + ", test, angular:" + f'{target_action:0.010f}' + ", angle_error:" + f'{angle_error:.015f}' + ", red point sum:" + str(self.count))
        self.episode += 1
        self.vel.linear.x = 0.2
        self.vel.angular.z = target_action
        self.nav_pub.publish(self.vel)
        if self.flag:
            if angle_error > self.threshold:
                self.draw_circle(self.vis_x, self.vis_y)
                self.count += 1
            self.draw_line(self.vis_x, self.vis_y, self.old_vis_x, self.old_vis_y)
        self.crop_img = self.image_resize.copy()
        self.crop_img = self.crop_img[301:600, 0:600]
        self.old_pos_x = self.pos_x
        self.old_pos_y = self.pos_y
        cv2.putText(self.crop_img, text = 'red point sum:' + str(self.count), org = (5, 25), fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color = (0,0,255))
        cv2.putText(self.crop_img, text = 'threshold:' + str(self.threshold), org = (5, 40), fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color = (0,0,0))
        cv2.imshow("realtime_evaluation", self.crop_img)
        temp = copy.deepcopy(img)
        cv2.waitKey(1)

        if self.old_pos_x == self.pos_x:
            self.flag = False

if __name__ == '__main__':
    rg = nav_cloning_node()
    DURATION = 0.2
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()