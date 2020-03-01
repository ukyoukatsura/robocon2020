#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2
import random
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
import numpy as np
import math
from tf import TransformListener
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

camera_preview = True

camera_fov = 60.0
camera_width = 640.0

class MyRobot():
    def __init__(self):
        # bot name
        # robot_name = rospy.get_param('~robot_name')
        # self.name = robot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # move_base
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # lidar scan subscriber
        self.scan = LaserScan()
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)
        # camera subscriber
        self.img = None
        self.camera_preview = camera_preview
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
        # move_base
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # mode
        self.mode = 0
        # tf
        self._tf_listener = tf.TransformListener()
        # Marker
        self.marker_pub = rospy.Publisher('enemy_position', Marker, queue_size = 1)

    # RESPECT
    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.name + "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def lidarCallback(self, data):
        self.scan = data

    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        if self.camera_preview:
            cv2.imshow("Image window", self.img)
            cv2.waitKey(1)
        # 緑の検出
        hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV_FULL)
        bgrLower = np.array([80, 50, 50])
        bgrUpper = np.array([110, 255, 255])
        hsv_mask = cv2.inRange(hsv_img, bgrLower, bgrUpper)
        rects = []
        labels, contours, hierarchy = cv2.findContours(hsv_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        for i in range(0, len(contours)):
            if len(contours[i]) > 0:

                rect = contours[i]
                x, y, w, h = cv2.boundingRect(rect)
                rects.append([x,y,w,h])
        print(rects)
        if len(rects) != 0:
            self.mode = 1
            # rectsから緑全体をもってくる
            Max_left = 640.0
            Max_right = 0.0
            for i in rects:
                if (i[0] - (i[2] / 2.0)) <= Max_left:
                    Max_left = (i[0] - (i[2] / 2.0))
                if (i[0] + (i[2] / 2.0)) >= Max_right:
                    Max_right = (i[0] + (i[2] / 2.0))
            print(Max_left)
            print(Max_right)
            
            angle_left = (Max_left - (camera_width / 2.0)) * (camera_fov / camera_width)
            angle_right = (Max_right - (camera_width / 2.0)) * (camera_fov / camera_width)
            
            angle_adjust = angle_left + angle_right
            
            # robot正面から何度の方向に緑の物体が存在するか計算
            angle = (rects[0][0] - (camera_width / 2.0)) * (camera_fov / camera_width)
            print("#####角度#####")
            print(angle)
            print(angle_adjust)
            # rectの大きさまで考慮する必要ありか？
            # lidarの点群からおおよその距離を算出
            if angle >= 0:
                distance = self.scan.ranges[int(angle)]
            else:
                distance = self.scan.ranges[int(359 + angle)]
            #if angle_adjust >= 0:
            #    distance = self.scan.ranges[int(angle_adjust)]
            #else:
            #    distance = self.scan.ranges[int(359 + angle_adjust)]
            print("#####距離#####")
            print(distance)
            # robotから見た座標値を算出　前がx軸、左がy軸
            robot_x = math.cos(math.radians(angle)) * distance
            robot_y = -math.sin(math.radians(angle)) * distance
            #robot_x = math.cos(math.radians(angle_adjust)) * distance
            #robot_y = -math.sin(math.radians(angle_adjust)) * distance
            print("#####x軸######")
            print(robot_x)
            print("#####y軸######")
            print(robot_y)
            
            ######要修正######
            
            # 地図座標系に変換
            #listener = tf.TransformListener()
            #listener.waitForTransform("/red_bot/map","/red_bot/base_footprint",rospy.Time(0),rospy.Duration(4.0))
            laser_point = PointStamped()
            laser_point.header.frame_id = "/red_bot/base_link"
            laser_point.header.stamp = rospy.Time(0)
            laser_point.point.x = robot_x
            laser_point.point.y = robot_y
            laser_point.point.z = 0.0
            p = PointStamped()
            p = self._tf_listener.transformPoint("/red_bot/map", laser_point)
            # 方向と位置をゴールとして指定
            # 一旦方向は無視して位置でデバッグ
            print("#####x_map#####")
            print(p.point.x)
            print("#####y_map#####")
            print(p.point.y)
            #self.setGoal(p.point.x,p.point.y,0)
            marker_data = Marker()
            marker_data.header.frame_id = "/red_bot/map"
            marker_data.header.stamp = rospy.Time.now()
            marker_data.ns = "text"
            marker_data.id = 0
            marker_data.action = Marker.ADD
            marker_data.type = 9
            marker_data.text = "object"
            marker_data.pose.position.x = p.point.x
            marker_data.pose.position.y = p.point.y
            marker_data.pose.position.z = 0.0
            marker_data.pose.orientation.x = 0.0
            marker_data.pose.orientation.y = 0.0
            marker_data.pose.orientation.z = 0.0
            marker_data.pose.orientation.w = 0.0
            marker_data.color.r = 1.0
            marker_data.color.g = 0.0
            marker_data.color.b = 0.0
            marker_data.color.a = 1.0
            marker_data.scale.x = 1.0
            marker_data.scale.y = 0.1
            marker_data.scale.z = 0.1
            self.marker_pub.publish(marker_data)
            
            # Goal Setting
            goal = PoseStamped()
            #goal.header.seq = self.goal_seq_no
            goal.header.frame_id = self.name + "/map"         # mapで座標系で指定する
            goal.header.stamp = rospy.Time.now()       # タイムスタンプは今の時間

            # ** 位置座標
            goal.pose.position.x = p.point.x
            goal.pose.position.y = p.point.y
            goal.pose.position.z = 0
            # ** 回転方向
            # オイラー角をクォータニオンに変換・設定する
            # RESPECT @hotic06 オイラー角をクォータニオンに変換・設定する
            #euler_val = self.orientstr_to_val(pos_list[2])
            #quate = tf.transformations.quaternion_from_euler(0.0, 0.0, euler_val)
            goal.pose.orientation.x = 0
            goal.pose.orientation.y = 0
            goal.pose.orientation.z = 0
            goal.pose.orientation.w = 0
            # debug
            print(goal)
            # 実際にTopicを配信する
            self.pub_goal.publish(goal)
            

    def basic_move(self):
        self.setGoal(-0.5,0,0)
        self.setGoal(-0.5,0,3.1415*0.25)
        
        self.setGoal(0,0.5,0)
        self.setGoal(0,0.5,3.1415*1.5)
        self.setGoal(0,0.5,3.1415)
        self.setGoal(0,0.5,3.1415*1.75)
        
        self.setGoal(0.5,0,3.1415*1.5)
        self.setGoal(0.5,0,3.1415)
        self.setGoal(0.5,0,3.1415*1.25)
        
        self.setGoal(0,-0.5,3.1415)
        self.setGoal(0,-0.5,3.1415*0.5)
        self.setGoal(0,-0.5,0)

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps
        while not rospy.is_shutdown():
            if self.mode == 0:
                self.basic_move()
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('my_robot_run')
    bot = MyRobot()
    bot.strategy()

