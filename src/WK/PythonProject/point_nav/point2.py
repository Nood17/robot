#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导航点 2（电梯）。坐标/音频原样保留，逻辑见 point_nav.py。"""
from point_nav import NavPointPlayer, THRESHOLD
import rospy

if __name__ == "__main__":
    rospy.init_node("nav_point_player")
    NavPointPlayer(-12.414095878601074, 20.286962509155273, -1.527,
                   "/home/zhuo/point_nav/audio/dianti.mp3", THRESHOLD)
    rospy.spin()
