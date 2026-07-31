#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导航点 1（卫生间）。坐标/音频原样保留，逻辑见 point_nav.py。"""
from point_nav import NavPointPlayer, THRESHOLD
import rospy

if __name__ == "__main__":
    rospy.init_node("nav_point_player")
    NavPointPlayer(-8.44294261932373, 8.894258499145508, 0,
                   "/home/zhuo/point_nav/audio/weishengjian.mp3", THRESHOLD)
    rospy.spin()
