#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导航点 4（讲解4）。坐标/音频原样保留，逻辑见 point_nav.py。"""
from point_nav import NavPointPlayer, THRESHOLD
import rospy

if __name__ == "__main__":
    rospy.init_node("nav_point_player")
    NavPointPlayer(0.84462, 1.21684, 2.63, "audio/introduce_4.mp3", THRESHOLD)
    rospy.spin()
