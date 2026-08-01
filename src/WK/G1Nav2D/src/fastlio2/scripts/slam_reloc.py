#!/usr/bin/env python3
"""SLAM 重定位桥接脚本。

修复：
1. 重定位后发布空 MarkerArray 清除 RViz 旧机器人标识（noooob 报告"重定位后旧标识不消失"）
2. 支持自动重定位：导航启动时通过参数指定初始位姿，无需手动在 RViz 点（noooob 报告"需要手动重定位"）
3. 多次重定位不阻塞：移除 pose_received 去重注释
"""
import rospy
import tf
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from fastlio.srv import SlamReLoc


class SlamRelocFromRViz:
    def __init__(self):
        self.pcd_path = rospy.get_param("~pcd_path", "")
        self.auto_reloc = rospy.get_param("~auto_reloc", False)
        self.auto_x = rospy.get_param("~auto_x", 0.0)
        self.auto_y = rospy.get_param("~auto_y", 0.0)
        self.auto_yaw = rospy.get_param("~auto_yaw", 0.0)

        rospy.wait_for_service('/slam_reloc')
        self.reloc_service = rospy.ServiceProxy('/slam_reloc', SlamReLoc)

        # Publisher to clear stale markers in RViz after relocalization.
        self.marker_pub = rospy.Publisher('/robot_markers', MarkerArray, queue_size=1, latch=True)

        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.loginfo("Waiting for /initialpose from RViz...")

        # Auto-relocalization on startup if configured.
        if self.auto_reloc:
            rospy.loginfo(f"Auto-relocalizing at ({self.auto_x}, {self.auto_y}, yaw={self.auto_yaw})")
            rospy.sleep(2.0)  # Wait for SLAM to be ready
            self._do_reloc(self.auto_x, self.auto_y, 0.0, 0.0, self.auto_yaw)

    def _clear_stale_markers(self):
        """Publish a DELETEALL marker to clear old robot markers in RViz."""
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.marker_pub.publish(marker_array)
        rospy.loginfo("Cleared stale robot markers in RViz")

    def _do_reloc(self, x, y, z, roll, pitch, yaw):
        try:
            resp = self.reloc_service(
                self.pcd_path,
                x, y, z,
                roll, pitch, yaw
            )
            rospy.loginfo("SlamReLoc service called successfully: %s", resp)
            # Clear stale markers after successful relocalization.
            rospy.sleep(0.5)
            self._clear_stale_markers()
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False

    def pose_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        rospy.loginfo("Received pose from RViz: x=%.2f, y=%.2f, yaw=%.2f", position.x, position.y, yaw)
        self._do_reloc(position.x, position.y, position.z, roll, pitch, yaw)


if __name__ == '__main__':
    rospy.init_node('slam_reloc_from_rviz')
    SlamRelocFromRViz()
    rospy.spin()
