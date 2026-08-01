#!/usr/bin/env python3
"""
G1 动作控制器基类。

将 g1_action.py / g1_action_plus.py / g1_action_time_adjust_limit.py 三个变体
中共享的骨架（关节索引、SDK 初始化、状态回调、权重控制、姿态插值、动作序列
6 阶段）提取到此处。各变体只需继承并覆盖以下钩子，保留各自具体数值不变：

  - configure_params()              : joint_limits 表（None 表示不限位）、time_buffer
  - clamp_poses(target_poses)       : 关节限位修正（基类默认直通）
  - get_pose_duration(current, target) : 返回 move_to_pose 的 duration（None=自动）
  - get_peace_poses(half_pi)        : 比耶动作关节角（各变体不同）
  - hold(duration)                  : 保持动作（基类 time.sleep，子类可改 hold_pose）

入口统一用 main(ControllerClass)。
"""
import sys
import time
import math

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_


# ==================== 常量定义 ====================
TOPIC_ARM_SDK = "rt/arm_sdk"
TOPIC_LOWSTATE = "rt/lowstate"
CTRL_DT = 0.02  # 20ms 控制周期


class JointIndex:
    # 左腿
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnkle = 4
    LeftAnkleRoll = 5
    # 右腿
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnkle = 10
    RightAnkleRoll = 11
    # 腰部
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
    # 左臂
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbowPitch = 18
    LeftElbowRoll = 19
    # 右臂
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbowPitch = 25
    RightElbowRoll = 26
    # 权重控制
    NotUsedJoint = 29


class G1ActionController:
    def __init__(self, network_interface, skip_channel_init=False):
        print("=" * 50)
        print(f"Initializing {self.__class__.__name__}...")
        print("=" * 50)

        if not skip_channel_init:
            ChannelFactoryInitialize(0, network_interface)

        self.publisher = ChannelPublisher(TOPIC_ARM_SDK, LowCmd_)
        self.publisher.Init()

        self.subscriber = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
        self.subscriber.Init(self._state_callback)

        self.current_state = None
        self.weight = 0.0
        self.crc = CRC()

        # 通用刚度参数
        self.kp_upper = 60.0
        self.kd_upper = 1.5
        self.kp_lower = 80.0
        self.kd_lower = 2.0
        self.max_joint_velocity = 0.5

        # 关节分组
        self.arm_joints = [
            JointIndex.LeftShoulderPitch, JointIndex.LeftShoulderRoll,
            JointIndex.LeftShoulderYaw, JointIndex.LeftElbowPitch,
            JointIndex.LeftElbowRoll,
            JointIndex.RightShoulderPitch, JointIndex.RightShoulderRoll,
            JointIndex.RightShoulderYaw, JointIndex.RightElbowPitch,
            JointIndex.RightElbowRoll,
            JointIndex.WaistYaw, JointIndex.WaistRoll, JointIndex.WaistPitch
        ]
        self.leg_joints = [
            JointIndex.LeftHipPitch, JointIndex.LeftHipRoll,
            JointIndex.LeftHipYaw, JointIndex.LeftKnee,
            JointIndex.LeftAnkle, JointIndex.LeftAnkleRoll,
            JointIndex.RightHipPitch, JointIndex.RightHipRoll,
            JointIndex.RightHipYaw, JointIndex.RightKnee,
            JointIndex.RightAnkle, JointIndex.RightAnkleRoll
        ]

        # 子类设置各自参数（joint_limits、time_buffer 等）
        self.configure_params()

        print("🚀 初始化完成，等待状态同步...")
        time.sleep(0.5)

    # ========== 子类可覆盖的钩子 ==========

    def configure_params(self):
        """子类设置 joint_limits（dict 或 None）、time_buffer 等。"""
        self.joint_limits = None

    def clamp_poses(self, target_poses):
        """关节限位修正。基类默认直通（无限位）。"""
        return target_poses

    def get_pose_duration(self, current_poses, target_poses, min_duration=2.0):
        """返回 move_to_pose 的 duration。None 表示子类自行处理。"""
        return None  # 默认由调用方指定 duration

    def get_peace_poses(self, half_pi):
        """比耶动作关节角。子类必须实现。"""
        raise NotImplementedError

    def hold(self, duration=2.0):
        """保持动作。基类用 time.sleep，子类可覆盖为 hold_pose。"""
        time.sleep(duration)

    # ========== 共享方法 ==========

    def _state_callback(self, msg: LowState_):
        self.current_state = msg

    def _send_cmd(self, msg: LowCmd_):
        msg.crc = self.crc.Crc(msg)
        self.publisher.Write(msg)

    def _create_empty_cmd(self):
        return unitree_hg_msg_dds__LowCmd_()

    def get_current_positions(self):
        positions = {}
        if self.current_state is None:
            return positions
        for i in range(len(self.current_state.motor_state)):
            positions[i] = self.current_state.motor_state[i].q
        return positions

    def smooth_interpolate(self, current, target, max_delta):
        delta = target - current
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
        return current + delta

    def set_weight(self, weight, duration=2.0):
        steps = int(duration / CTRL_DT)
        delta = (weight - self.weight) / steps
        msg = self._create_empty_cmd()
        for i in range(steps):
            self.weight += delta
            msg.motor_cmd[JointIndex.NotUsedJoint].q = self.weight
            current_pos = self.get_current_positions()
            for joint_idx in range(29):
                if joint_idx in current_pos:
                    msg.motor_cmd[joint_idx].q = current_pos[joint_idx]
                msg.motor_cmd[joint_idx].dq = 0.0
                msg.motor_cmd[joint_idx].kp = self.kp_lower
                msg.motor_cmd[joint_idx].kd = self.kd_lower
                msg.motor_cmd[joint_idx].tau = 0.0
            self._send_cmd(msg)
            time.sleep(CTRL_DT)
        self.weight = weight

    def move_to_pose(self, target_poses, duration=3.0, min_duration=2.0):
        """移动到目标姿态。duration=None 时用 get_pose_duration 自动计算。"""
        # 限位修正
        target_poses = self.clamp_poses(target_poses)

        # 自动时长
        if duration is None:
            duration = self.get_pose_duration(self.get_current_positions(),
                                              target_poses, min_duration)
            if duration is None:
                duration = 3.0  # 兜底

        steps = int(duration / CTRL_DT)
        max_delta = self.max_joint_velocity * CTRL_DT
        start_poses = self.get_current_positions()
        current_poses = {k: v for k, v in start_poses.items()}
        msg = self._create_empty_cmd()

        for step in range(steps):
            for joint_idx, target_angle in target_poses.items():
                if joint_idx in current_poses:
                    current_poses[joint_idx] = self.smooth_interpolate(
                        current_poses[joint_idx], target_angle, max_delta)
            for joint_idx in range(29):
                if joint_idx in current_poses:
                    msg.motor_cmd[joint_idx].q = current_poses[joint_idx]
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

        self._print_pose_error(target_poses)
        return current_poses

    def _print_pose_error(self, target_poses):
        """姿态到达误差检查（基类提供，子类可覆盖）。"""
        print("\n--- 姿态到达检查 ---")
        final_positions = self.get_current_positions()
        for joint_idx, target_angle in target_poses.items():
            if joint_idx in final_positions:
                error = abs(final_positions[joint_idx] - target_angle)
                if error > 0.1:
                    print(f"  关节{joint_idx}: 目标={target_angle:.2f}, 实际={final_positions[joint_idx]:.2f}, 误差={error:.2f}")
        print("-------------------\n")

    def run_action_sequence(self):
        """执行动作序列（6 阶段）。子类通常不需覆盖。"""
        print("\n" + "=" * 50)
        print("开始动作序列")
        print("=" * 50)

        # 等待状态数据
        print("等待机器人状态数据...")
        timeout = 5.0
        start_time = time.time()
        while self.current_state is None:
            time.sleep(0.1)
            if time.time() - start_time > timeout:
                print("[ERROR] 未收到状态数据！请检查网络连接或 DDS 话题。")
                return

        current = self.get_current_positions()
        print("当前关节位置已读取")
        half_pi = math.pi / 2

        # 阶段 1: 获取控制权
        print("\n[1/5] 获取控制权...")
        self.set_weight(1.0, duration=2.0)

        # 阶段 2: 初始化姿态（归零）
        print("\n[2/5] 初始化姿态...")
        init_poses = {idx: 0.0 for idx in self.arm_joints}
        init_poses.update({idx: current[idx] for idx in self.leg_joints})
        self.move_to_pose(init_poses, duration=2.0)

        # 阶段 3: 抬起手臂
        print("\n[3/5] 抬起手臂...")
        raise_poses = {
            JointIndex.LeftShoulderPitch: 0.0,
            JointIndex.LeftShoulderRoll: half_pi,
            JointIndex.LeftShoulderYaw: 0.0,
            JointIndex.LeftElbowPitch: half_pi,
            JointIndex.LeftElbowRoll: 0.0,
            JointIndex.RightShoulderPitch: 0.0,
            JointIndex.RightShoulderRoll: -half_pi,
            JointIndex.RightShoulderYaw: 0.0,
            JointIndex.RightElbowPitch: half_pi,
            JointIndex.RightElbowRoll: 0.0,
            JointIndex.WaistYaw: 0.0,
            JointIndex.WaistRoll: 0.0,
            JointIndex.WaistPitch: 0.0,
        }
        raise_poses.update({idx: current[idx] for idx in self.leg_joints})
        self.move_to_pose(raise_poses, duration=3.0)

        # 阶段 4: 比耶动作（子类提供 poses）
        print("\n[4/5] 比耶动作...")
        peace_poses = self.get_peace_poses(half_pi)
        peace_poses.update({idx: current[idx] for idx in self.leg_joints})
        peace_duration = self.get_pose_duration(current, peace_poses)
        if peace_duration is not None:
            self.move_to_pose(peace_poses, duration=peace_duration)
        else:
            self.move_to_pose(peace_poses, duration=2.0)

        # 保持动作
        print("保持动作...")
        self.hold(2.0)

        # 阶段 5: 放下手臂
        print("\n[5/5] 放下手臂...")
        final_poses = {idx: 0.0 for idx in self.arm_joints}
        final_poses.update({idx: current[idx] for idx in self.leg_joints})
        self.move_to_pose(final_poses, duration=3.0)

        # 释放控制权
        print("\n释放控制权...")
        self.set_weight(0.0, duration=2.0)
        print("\n动作序列完成！")


def main(controller_class):
    """共享入口。"""
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <network_interface>")
        sys.exit(1)
    network_interface = sys.argv[1]
    controller = controller_class(network_interface)
    input("按 Enter 开始执行动作...")
    controller.run_action_sequence()
