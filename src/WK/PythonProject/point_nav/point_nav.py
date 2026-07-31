#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单点导航 + 音频播放器。

point1..point5 五个脚本共享的 NavPointPlayer 类。各 point 脚本仅
__main__ 常量（坐标 / 音频路径）不同，故统一到此处。

用法：
  python3 point_nav.py <point_id> [network_interface]
  point_id ∈ {1,2,3,4,5}，对应原 point1..point5.py 的坐标与音频。
"""
import rospy
import math
import os
import threading
import tf.transformations as tft
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
from playsound import playsound


# 各导航点数据（原 point1..point5.py __main__ 常量，原样保留）
POINTS = {
    1: {"x": -8.44294261932373, "y": 8.894258499145508, "theta": 0,
        "audio": "/home/zhuo/point_nav/audio/weishengjian.mp3"},
    2: {"x": -12.414095878601074, "y": 20.286962509155273, "theta": -1.527,
        "audio": "/home/zhuo/point_nav/audio/dianti.mp3"},
    3: {"x": 2.0573, "y": -1.1618, "theta": -2.63,
        "audio": "audio/introduce_3.mp3"},
    4: {"x": 0.84462, "y": 1.21684, "theta": 2.63,
        "audio": "audio/introduce_4.mp3"},
    5: {"x": 0.5240, "y": 3.5919, "theta": 1.06,
        "audio": "audio/introduce_5.mp3"},
}
THRESHOLD = 0.5


class NavPointPlayer:
    def __init__(self, target_x, target_y, target_theta, audio_file, threshold=THRESHOLD):
        self.target_x = target_x
        self.target_y = target_y
        self.target_theta = target_theta
        self.audio_file = audio_file
        self.threshold = threshold
        self.reached = False

        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        self.feedback_sub = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.feedback_callback)

        rospy.sleep(1.0)  # 等待topic连接
        self.publish_goal()

    def publish_goal(self):
        goal = MoveBaseActionGoal()
        goal.goal.target_pose.header.frame_id = "map"
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.pose.position.x = self.target_x
        goal.goal.target_pose.pose.position.y = self.target_y
        q = tft.quaternion_from_euler(0, 0, self.target_theta)
        goal.goal.target_pose.pose.orientation.x = q[0]
        goal.goal.target_pose.pose.orientation.y = q[1]
        goal.goal.target_pose.pose.orientation.z = q[2]
        goal.goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo(f"[导航目标] x={self.target_x}, y={self.target_y}, θ={self.target_theta}")
        self.goal_pub.publish(goal)

    def feedback_callback(self, msg):
        if self.reached:
            return
        current_pose = msg.feedback.base_position.pose
        dx = current_pose.position.x - self.target_x
        dy = current_pose.position.y - self.target_y
        dist = math.hypot(dx, dy)
        rospy.loginfo_throttle(2, f"[当前位置] ({current_pose.position.x:.2f}, {current_pose.position.y:.2f}) -> 距离目标 {dist:.2f} m")
        if dist <= self.threshold:
            rospy.loginfo("[到达目标] 播放音频...")
            self.reached = True
            self.play_audio()

    def play_audio(self):
        def _play():
            abs_path = os.path.abspath(self.audio_file)
            try:
                playsound(abs_path)
                rospy.loginfo("[完成] 音频播放结束，退出程序。")
                rospy.signal_shutdown("任务完成")
            except Exception as e:
                rospy.logerr(f"[错误] 音频播放失败: {e}")
        t = threading.Thread(target=_play)
        t.start()


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 point_nav.py <point_id>  (point_id: 1-5)")
        sys.exit(1)
    point_id = int(sys.argv[1])
    if point_id not in POINTS:
        print(f"Invalid point_id {point_id}. Valid: {list(POINTS.keys())}")
        sys.exit(1)

    p = POINTS[point_id]
    rospy.init_node("nav_point_player")
    node = NavPointPlayer(p["x"], p["y"], p["theta"], p["audio"])
    rospy.spin()
