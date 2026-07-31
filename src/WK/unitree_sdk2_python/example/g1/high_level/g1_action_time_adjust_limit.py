#!/usr/bin/env python3
"""G1 动作控制器（时限调整版）。静默限位+详细列表、time_buffer=1.0、全程自动时长。

继承 G1ActionController，覆盖：
  - configure_params: joint_limits 表（同 plus）+ time_buffer=1.0
  - clamp_poses: 静默限位 + verbose 修正列表
  - hold: hold_pose（持续发命令防下垂）
  - get_pose_duration: calculate_duration（行程/速度 + time_buffer，带打印）
  - get_peace_poses: LeftElbowPitch=0.0, RightElbowPitch=-1.5
注意：本变体所有 move_to_pose 均自动计算时长（duration=None）。
"""
import time
from g1_action_base import (G1ActionController as _Base, JointIndex,
                            CTRL_DT)


class G1ActionController(_Base):
    def configure_params(self):
        self.joint_limits = {
            # 左腿
            JointIndex.LeftHipPitch: (-2.35, 2.35),
            JointIndex.LeftHipRoll: (-0.5, 0.5),
            JointIndex.LeftHipYaw: (-0.5, 0.5),
            JointIndex.LeftKnee: (-2.5, 2.5),
            JointIndex.LeftAnkle: (-0.8, 0.8),
            JointIndex.LeftAnkleRoll: (-0.5, 0.5),
            # 右腿
            JointIndex.RightHipPitch: (-2.35, 2.35),
            JointIndex.RightHipRoll: (-0.5, 0.5),
            JointIndex.RightHipYaw: (-0.5, 0.5),
            JointIndex.RightKnee: (-2.5, 2.5),
            JointIndex.RightAnkle: (-0.8, 0.8),
            JointIndex.RightAnkleRoll: (-0.5, 0.5),
            # 腰部
            JointIndex.WaistYaw: (-1.0, 1.0),
            JointIndex.WaistRoll: (-0.5, 0.5),
            JointIndex.WaistPitch: (-0.5, 0.5),
            # 左臂
            JointIndex.LeftShoulderPitch: (-3.14, 2.09),
            JointIndex.LeftShoulderRoll: (-0.5, 3.14),
            JointIndex.LeftShoulderYaw: (-2.09, 2.09),
            JointIndex.LeftElbowPitch: (-2.5, 0.0),
            JointIndex.LeftElbowRoll: (-1.57, 1.57),
            # 右臂
            JointIndex.RightShoulderPitch: (-3.14, 2.09),
            JointIndex.RightShoulderRoll: (-3.14, 0.5),
            JointIndex.RightShoulderYaw: (-2.09, 2.09),
            JointIndex.RightElbowPitch: (-2.5, 0.0),
            JointIndex.RightElbowRoll: (-1.57, 1.57),
        }
        self.time_buffer = 1.0   # 时间缓冲（秒）

    def clamp_to_limits(self, joint_idx, angle):
        """静默限位（不逐关节打印）。"""
        if joint_idx in self.joint_limits:
            lower, upper = self.joint_limits[joint_idx]
            if angle < lower:
                return lower
            elif angle > upper:
                return upper
        return angle

    def clamp_poses(self, target_poses):
        """检查并修正所有目标角度（verbose 修正列表）。"""
        clamped_poses = {}
        clamped_joints = []
        for joint_idx, target_angle in target_poses.items():
            original = target_angle
            clamped_angle = self.clamp_to_limits(joint_idx, target_angle)
            if abs(clamped_angle - original) > 0.01:
                clamped_joints.append((joint_idx, original, clamped_angle))
            clamped_poses[joint_idx] = clamped_angle
        if clamped_joints:
            print("  ⚠️ 关节限位修正:")
            for joint_idx, original, clamped in clamped_joints:
                print(f"    关节{joint_idx}: {original:.2f} → {clamped:.2f}")
        return clamped_poses

    def hold(self, duration=2.0):
        """hold_pose：持续发送命令防止下垂。"""
        steps = int(duration / CTRL_DT)
        msg = self._create_empty_cmd()
        for _ in range(steps):
            current_pos = self.get_current_positions()
            for joint_idx in range(29):
                if joint_idx in current_pos:
                    msg.motor_cmd[joint_idx].q = current_pos[joint_idx]
                if joint_idx in self.leg_joints:
                    msg.motor_cmd[joint_idx].kp = self.kp_lower
                    msg.motor_cmd[joint_idx].kd = self.kd_lower
                elif joint_idx in self.arm_joints:
                    msg.motor_cmd[joint_idx].kp = self.kp_upper
                    msg.motor_cmd[joint_idx].kd = self.kd_upper
                else:
                    msg.motor_cmd[joint_idx].kp = self.kp_upper
                    msg.motor_cmd[joint_idx].kd = self.kd_upper
                msg.motor_cmd[joint_idx].dq = 0.0
                msg.motor_cmd[joint_idx].tau = 0.0
            msg.motor_cmd[JointIndex.NotUsedJoint].q = self.weight
            self._send_cmd(msg)
            time.sleep(CTRL_DT)

    def get_pose_duration(self, current_poses, target_poses, min_duration=2.0):
        """根据关节位移自动计算所需时间（行程/速度 + time_buffer，带打印）。"""
        max_travel = 0.0
        max_joint = None
        for joint_idx, target_angle in target_poses.items():
            if joint_idx in current_poses:
                travel = abs(target_angle - current_poses[joint_idx])
                if travel > max_travel:
                    max_travel = travel
                    max_joint = joint_idx
        if max_travel < 0.01:
            return min_duration
        needed_time = max_travel / self.max_joint_velocity + self.time_buffer
        duration = max(needed_time, min_duration)
        print(f"  ⏱️ 时间估计: 最大行程 {max_travel:.2f}rad (关节{max_joint}), 需要 {duration:.1f}秒")
        return duration

    def get_peace_poses(self, half_pi):
        return {
            JointIndex.LeftShoulderPitch: 0.0,
            JointIndex.LeftShoulderRoll: -0.50,
            JointIndex.LeftShoulderYaw: 0.0,
            JointIndex.LeftElbowPitch: 0.0,
            JointIndex.LeftElbowRoll: 0.0,
            # 右臂比耶
            JointIndex.RightShoulderPitch: -half_pi * 2,
            JointIndex.RightShoulderRoll: -half_pi,
            JointIndex.RightShoulderYaw: 0.0,
            JointIndex.RightElbowPitch: -1.5,
            JointIndex.RightElbowRoll: 1.0,
            JointIndex.WaistYaw: 0.0,
            JointIndex.WaistRoll: 0.0,
            JointIndex.WaistPitch: 0.0,
        }


if __name__ == "__main__":
    from g1_action_base import main
    main(G1ActionController)
