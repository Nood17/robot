#!/usr/bin/env python3
"""
G1 控制器基类。

将 7 个 g1_control_*.py 变体中共享的骨架（SDK 初始化、订阅、
控制定时器、到位锁定、__main__ 入口）提取到此处。各变体只需继承并
重写以下策略钩子，保留各自的全部具体数值不变：

  - configure_params()        : 设置 max_v / max_acc / 权重 / PID 增益等
  - setup_feedback()          : DDS 里程计 / ROS /odom / 无反馈
  - solve_step(...)           : 单轴求解（OSQP / EMA / PID）
  - process_cmd_vel(msg)      : cmd_vel 预处理（锁定守卫 / 禁倒车 / 角速度限幅）
  - on_solve_failure(v_last)  : 求解失败策略（返 0 或保持 v_last）

锁定机制通过 self.lock_style 配置：
  "latch"   : 锁存 + Move(0,0,0)（mpc 家族）
  "counter" : 计数器定时锁定（openloop）
  "none"    : 无锁定（debug / pid）
"""
import rospy
import sys
import threading
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


class G1BaseController:
    # ---- 子类可覆盖的类属性 ----
    node_name = "unitree_g1_controller"
    lock_style = "latch"            # "latch" | "counter" | "none"
    use_stopmove_deadband = True    # True: |cmd|<0.01 用 StopMove；False: 总是 Move
    can_move_initial = False

    def __init__(self, network_interface):
        rospy.loginfo("========================================")
        rospy.loginfo(f"Initializing {self.__class__.__name__}...")
        rospy.loginfo("========================================")

        # 1. SDK 初始化（统一 try/except，原 openloop/pid 无保护，加上不损失行为）
        try:
            ChannelFactoryInitialize(0, network_interface)
        except Exception as e:
            print(f"[ERROR] 网络初始化失败: {e}")
            sys.exit(-1)

        self.sport_client = LocoClient()
        self.sport_client.SetTimeout(10.0)
        try:
            self.sport_client.Init()
        except Exception as e:
            print(f"[ERROR] 机器人连接失败: {e}")
            sys.exit(-1)

        # 2. 通用状态
        self.control_freq = 50.0
        self.dt = 1.0 / self.control_freq
        self.can_move = self.can_move_initial
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_wz = 0.0

        # 3. 锁定机制状态
        self.is_stopped = False
        self.stop_velocity_threshold = 0.03   # latch 模式默认；openloop 子类覆盖为 0.02
        # counter 模式专用
        self.stop_counter = 0
        self.stop_threshold_cycles = 100

        # 4. 子类设置各自参数
        self.configure_params()

        # 5. 订阅
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.path_sub = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.path_callback)

        # 6. 子类设置反馈源（DDS / ROS /odom / 无）
        self.setup_feedback()

        # 7. 控制定时器
        self.control_timer = rospy.Timer(rospy.Duration(self.dt), self.control_loop)

        rospy.loginfo(f"🚀 {self.__class__.__name__} 已启动")

    # ========== 子类必须实现 / 可覆盖的钩子 ==========

    def configure_params(self):
        """子类设置 max_v / max_acc / 权重 / 增益 / 锁定阈值等。"""
        pass

    def setup_feedback(self):
        """子类订阅 DDS 里程计或 ROS /odom，写入 self.current_*。"""
        pass

    def solve_step(self, v_current, v_target, v_last_cmd, max_v, max_acc, **kw):
        """单轴求解，返回命令速度。子类必须实现。"""
        raise NotImplementedError

    def on_solve_failure(self, v_last_cmd):
        """OSQP 等求解失败时的返回值。默认 0.0；部分子类返回 v_last_cmd。"""
        return 0.0

    def process_cmd_vel(self, msg):
        """cmd_vel 预处理：禁倒车 / 角速度限幅等。默认直通。"""
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_wz = msg.angular.z

    # ========== 共享回调 ==========

    def path_callback(self, msg: Path):
        self.can_move = len(msg.poses) > 0
        # Track route health: detect path oscillation (route becoming weird).
        new_len = len(msg.poses)
        if hasattr(self, '_last_path_len') and self._last_path_len > 0:
            # Path length jumping wildly = TEB replanning frantically = route issue
            if abs(new_len - self._last_path_len) > max(5, self._last_path_len * 0.5):
                if not hasattr(self, '_path_oscillation_count'):
                    self._path_oscillation_count = 0
                self._path_oscillation_count += 1
                if self._path_oscillation_count > 3:
                    rospy.logwarn("⚠️ 路径震荡检测：TEB 频繁重规划，可能路线诡异")
                    self._path_oscillation_count = 0
        self._last_path_len = new_len

    def cmd_vel_callback(self, msg: Twist):
        # 锁定守卫（latch / counter 模式）
        if self.is_stopped and self.lock_style in ("latch", "counter"):
            if (abs(msg.linear.x) < 0.05 and abs(msg.linear.y) < 0.05
                    and abs(msg.angular.z) < 0.1):
                return
            else:
                self.is_stopped = False
                if self.lock_style == "counter":
                    self.stop_counter = 0
                rospy.loginfo("🔓 解锁：开始新运动")
        self.process_cmd_vel(msg)

    # ========== 锁定判定 ==========

    def _check_lock(self):
        """到位锁定检测。增加确认计数器，防止里程计延迟导致提前锁定（定点偏移根因）。"""
        if self.lock_style == "none":
            return
        target_thresh = 0.001 if self.lock_style == "counter" else 0.01
        # Use stricter velocity threshold + confirmation count to avoid premature lock.
        strict_vel_thresh = 0.015  # Stricter than stop_velocity_threshold
        if (abs(self.target_vx) < target_thresh and
                abs(self.target_vy) < target_thresh and
                abs(self.target_wz) < target_thresh and
                abs(self.current_vx) < strict_vel_thresh and
                abs(self.current_vy) < strict_vel_thresh and
                abs(self.current_wz) < strict_vel_thresh):
            # Require N consecutive confirmations to actually lock (fixes offset drift).
            if not hasattr(self, '_lock_confirm_count'):
                self._lock_confirm_count = 0
            self._lock_confirm_count += 1
            if self._lock_confirm_count >= 5:  # 5 cycles = 100ms at 50Hz
                if not self.is_stopped:
                    rospy.loginfo("🔒 锁定：到达目标点，停止运动 (确认5次)")
                self.is_stopped = True
                if self.lock_style == "counter":
                    self.stop_counter = self.stop_threshold_cycles
        else:
            if hasattr(self, '_lock_confirm_count'):
                self._lock_confirm_count = 0
    # ========== 主控制循环（模板方法） ==========

    def control_loop(self, event):
        # 1. 路径安全门
        if not self.can_move:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0

        # 2. 锁定检测
        self._check_lock()

        # 3. 锁定态：强制零速
        if self.is_stopped:
            if self.lock_style == "counter" and self.stop_counter > 0:
                self.sport_client.Move(0.0, 0.0, 0.0)
                self.stop_counter -= 1
                if self.stop_counter == 0:
                    self.is_stopped = False
            else:
                self.sport_client.Move(0.0, 0.0, 0.0)
            self.last_cmd_vx = 0.0
            self.last_cmd_vy = 0.0
            self.last_cmd_wz = 0.0
            return

        # 4. 求解各轴命令（子类实现 solve_step）
        cmd_vx = self.solve_step(self.current_vx, self.target_vx, self.last_cmd_vx,
                                 self.max_vx, self.max_acc_v)
        cmd_vy = self.solve_step(self.current_vy, self.target_vy, self.last_cmd_vy,
                                 self.max_vy, self.max_acc_v)
        cmd_wz = self.solve_step(self.current_wz, self.target_wz, self.last_cmd_wz,
                                 self.max_wz, self.max_acc_w)

        # 4.5 连续加速度限幅：防止 solve_step 输出跳变导致顿挫（快一下顿一下根因）
        max_delta_v = 0.05  # 单步最大速度变化量 (m/s per 20ms step)
        max_delta_w = 0.08  # 单步最大角速度变化量 (rad/s per 20ms step)
        cmd_vx = np.clip(cmd_vx, self.last_cmd_vx - max_delta_v, self.last_cmd_vx + max_delta_v)
        cmd_vy = np.clip(cmd_vy, self.last_cmd_vy - max_delta_v, self.last_cmd_vy + max_delta_v)
        cmd_wz = np.clip(cmd_wz, self.last_cmd_wz - max_delta_w, self.last_cmd_wz + max_delta_w)

        # 5. 发送
        if self.use_stopmove_deadband and abs(cmd_vx) < 0.01 and abs(cmd_vy) < 0.01 and abs(cmd_wz) < 0.01:
            self.sport_client.StopMove()
        else:
            self.sport_client.Move(cmd_vx, cmd_vy, cmd_wz)

        self.last_cmd_vx = cmd_vx
        self.last_cmd_vy = cmd_vy
        self.last_cmd_wz = cmd_wz

    # ========== 共享入口 ==========

    @staticmethod
    def run(cls):
        if len(sys.argv) < 2:
            print(f"Usage: python3 {sys.argv[0]} <network_interface>")
            sys.exit(-1)
        network_interface = sys.argv[1]
        rospy.init_node(cls.node_name, anonymous=False)
        try:
            controller = cls(network_interface)
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
