#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导航点 3（讲解3）。坐标/音频原样保留，逻辑见 point_nav.py。"""
from point_nav import NavPointPlayer, THRESHOLD
import rospy

if __name__ == "__main__":
    rospy.init_node("nav_point_player")
    NavPointPlayer(2.0573, -1.1618, -2.63, "audio/introduce_3.mp3", THRESHOLD)
    rospy.spin()
