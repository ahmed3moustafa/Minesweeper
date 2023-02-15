#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import time
import random

def talker():
    pose = Pose2D()
    pose.x = 3
    pose.y = 14
    pose.theta = 2
    i=0
    pub = rospy.Publisher('navigation', Pose2D, queue_size = 20)
    rospy.init_node('map_sub', anonymous=True)
    rate = rospy.Rate(1)
    for i in range (10):
        pose.x=i+2
        pose.y= i+3
        pose.theta=random.randint(0,3)
        pub.publish(pose)
                # pose.x = 3 
                # pose.theta = 1
        rate.sleep()
talker()