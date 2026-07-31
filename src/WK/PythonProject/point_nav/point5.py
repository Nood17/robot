#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导航点 5（讲解5）。坐标/音频原样保留，逻辑见 point_nav.py。"""
from point_nav import NavPointPlayer, THRESHOLD
import rospy

if __name__ == "__main__":
    rospy.init_node("nav_point_player")
    NavPointPlayer(0.5240, 3.5919, 1.06, "audio/introduce_5.mp3", THRESHOLD)
    rospy.spin()
