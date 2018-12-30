#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import sys
import datetime
import numpy as np
import tf
from std_msgs.msg import *
from geometry_msgs.msg import *
from collections import OrderedDict
from roslib import message
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from math import copysign
from math import pi


def main():
    rospy.init_node('hsr_init_pos', anonymous=True)

    rospy.loginfo("Setting Initial Pose")
    init_pos_x  = rospy.get_param('~init_pos_x')
    init_pos_y  = rospy.get_param('~init_pos_y')
    init_pos_th = rospy.get_param('~init_pos_th')        

    rospy.loginfo("(x, y, th)=(%f, %f, %f)",init_pos_x,init_pos_y,init_pos_th)

    
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped)
    p   = PoseWithCovarianceStamped();
    msg = PoseWithCovariance();
    q_angle = tf.transformations.quaternion_from_euler(0.0, 0.0, init_pos_th, 'sxyz')
    q = Quaternion(*q_angle)
    msg.pose = Pose(Point(init_pos_x ,init_pos_y, 0.0), q); #初期位置
    msg.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853];
    p.pose = msg;
    p.header.stamp = rospy.Time.now()
    p.header.frame_id="map"
    rospy.sleep(2.0)
    rospy.loginfo("Setting Pose")
    pub.publish(p);


#    rate = rospy.Rate(10) # 10hz
#    rospy.loginfo("Setting Initial Pose")
#    while not rospy.is_shutdown():
#        rospy.loginfo("SSSS")
#        rate.sleep()
        
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
