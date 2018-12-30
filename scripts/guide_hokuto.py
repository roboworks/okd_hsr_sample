#HSRが北斗館を案内するデモ
# Hiroyuki Okada, 30 Dec 2018

#!/usr/bin/python
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
from hsrb_interface import Robot


# ロボット機能を使うための準備
robot = Robot()
base = robot.try_get('omni_base')
tts = robot.try_get('default_tts')
whole_body = robot.try_get('whole_body')

def go_and_say(pos=(0,0,0), contents=''):
    try:
        base.go_abs(pos[0], pos[1], pos[2], 180.0)
    except:
        rospy.logerr('Fail go')
    tts.say(contents)
    rospy.sleep(5)

_SENARIO = [
    ((2.9, 0.36, -1.57, 180,0), u'ここが僕のお気に入りのソファだ。くつろいでテレビが見れるよ。僕は座れないけどね。'),
    ((5.4, 0.07, -1.57, 180,0), u'ここからお台場の海が見えるよ。綺麗だね。'),
    ((5.7, 1.6, 3.14, 180,0), u'ここはIH式のレンジだ。何を作ろうかな。'),
    ((5.7, 2.6, 3.14, 180,0), u'ここがシンクだ。水は出ないけど。'),
    ((4.4, 5.8, -1.57, 180,0), u'ここでみんなで食事が出来るんだ。'),
    ((5.2, 6.3, 0.07, 180,0), u'これは近未来テレビ。何とジェスチャーで操作できるんだ。すごいね。'),
    ((1.5, 3.3, 1.57, 180,0), u'ここが僕の一番のおすすめスポットの気になる木。落ち着くな。'),
    ((0.0, 0.0, 3.14, 180,0), u'今日の説明はこれでおしまい。ばいばい。')]

if __name__=='__main__':
    rospy.init_node('guide_hokuto', anonymous=True)

    # 自己位置を設定
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


    # 初期姿勢に遷移
    try:
        whole_body.move_to_go()
    except:
        rospy.logerr('Fail move_to_neutral')

    # まずは一言
    tts.say(u'こんにちはHSRだよ。僕が北斗館を案内するね。')

    for unit in _SENARIO:
        go_and_say(unit[0], unit[1])
