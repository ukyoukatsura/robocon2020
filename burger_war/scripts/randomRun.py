#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is rumdom run node.
subscribe No topcs.
Publish 'cmd_vel' topic. 
mainly use for simple sample program

by Takuya Yamaguhi.
'''

import rospy
import random
import actionlib
import actionlib_msgs
from geometry_msgs.msg import PoseStamped
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist


class RandomBot():
    def __init__(self, bot_name="NoName"):
        # bot name 
        self.name = bot_name
        self.numgoal = 0
        # move_base
        self.pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1, latch=True)

    # RESPECT
    def setGoal(self,x,y,yaw):

        goal = PoseStamped()
        goal.header.seq = self.numgoal
        self.numgoal += 1
        #goal.target_pose.header.frame_id = self.name + "/map"
        goal.header.frame_id = "/map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

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
            print("test")
            self.basic_move()
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random')
    bot.strategy()

