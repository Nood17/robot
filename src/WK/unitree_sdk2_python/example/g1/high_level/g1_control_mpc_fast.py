#!/usr/bin/env python3
"""G1 自适应 MPC 控制器（不对称闭环 + 全局禁倒车 + 高/低速模式切换）。

继承 G1BaseController，覆盖：
  - configure_params: 分轴加速度、模式权重、死区
  - process_cmd_vel: 禁倒车 + 快速模式角速度限幅
  - solve_step: 不对称 OSQP（减速保持 v_current，加速用 0.6/0.4 混合）
  - control_loop: 加入 get_adaptive_mode 切换 + 调试打印
"""
import rospy
import numpy as np
import osqp
import scipy.sparse as sp
from geometry_msgs.msg import Twist

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

from g1_controller_base import G1BaseController


class AdaptiveMPCController(G1BaseController):
    node_name = "unitree_mpc_controller"

    def configure_params(self):
        self.max_vx = 1.5
        self.max_vy = 0.6
        self.max_wz = 1.5
        self.max_acc_v = 5.0
        self.max_acc_w_fast = 4.0
        self.max_acc_w_slow = 3.0
        self.Q_v = 10.0
        self.R_v_lin = 1.0
        self.R_v_ang_fast = 2.5
        self.R_v_ang_slow = 2.0
        self.adaptive_mode = "auto"
        self.deadzone_threshold = 0.05
        self.last_mode = "slow"
        self.mode_switch_count = 0
        self.stop_velocity_threshold = 0.03

    def setup_feedback(self):
        self.odom_dds_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
        self.odom_dds_sub.Init(self.dds_odom_callback)

    def dds_odom_callback(self, msg: SportModeState_):
        self.current_vx = msg.velocity[0]
        self.current_vy = msg.velocity[1]
        self.current_wz = msg.yaw_speed

    def get_adaptive_mode(self):
        if self.adaptive_mode != "auto":
            return self.adaptive_mode
        # 【修改】提高退出阈值，0.26 属于低速，应该切回 Slow
        enter_fast_threshold = 0.4
        exit_fast_threshold = 0.3  # 0.15 -> 0.3
        if self.last_mode == "fast":
            if self.target_vx < exit_fast_threshold:
                return "slow"
            else:
                return "fast"
        else:
            if self.target_vx > enter_fast_threshold:
                return "fast"
            else:
                return "slow"

    def process_cmd_vel(self, msg):
        # 全局禁止倒车
        if msg.linear.x < 0.0:
            self.target_vx = 0.0
            if self.last_cmd_vx >= 0:
                rospy.logwarn_throttle(1.0, "⛔ 拦截倒车指令")
        else:
            self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y

        mode = self.get_adaptive_mode()
        raw_wz = msg.angular.z
        if mode == "fast":
            if abs(raw_wz) < self.deadzone_threshold:
                self.target_wz = 0.0
            else:
                if self.current_vx > 0.2:
                    max_allowed_wz = 1.0 / (self.current_vx + 1.0)
                    max_allowed_wz = max(0.2, min(0.8, max_allowed_wz))
                    self.target_wz = np.clip(raw_wz, -max_allowed_wz, max_allowed_wz)
                else:
                    self.target_wz = raw_wz
        else:
            self.target_wz = raw_wz

    def solve_step(self, v_current, v_target, v_last_cmd, max_v, max_acc, **kw):
        # 1. 输入截断
        v_current = np.clip(v_current, -max_v * 1.2, max_v * 1.2)
        v_target = np.clip(v_target, -max_v, max_v)

        # 2. 不对称基点：减速时保持当前速度，加速时用 0.6/0.4 混合
        if v_current > v_target + 0.05:
            v_base = v_current
        else:
            v_base = 0.6 * v_current + 0.4 * v_last_cmd

        acc_limit = max_acc * self.dt
        lower_bound = max(-max_v, v_base - acc_limit)
        upper_bound = min(max_v, v_base + acc_limit)
        if lower_bound > upper_bound:
            lower_bound = -max_v
            upper_bound = max_v

        # 3. 选权重（角轴按模式，线轴用 R_v_lin）
        R_v = kw.get("R_v", self.R_v_lin)
        P = sp.csc_matrix([[2 * (self.Q_v + R_v)]])
        q = np.array([-2 * (self.Q_v * v_target + R_v * v_last_cmd)])
        A_box = sp.csc_matrix([[1.0]])
        prob = osqp.OSQP()
        prob.setup(P, q, A_box,
                   np.array([lower_bound]), np.array([upper_bound]),
                   verbose=False, eps_abs=1e-3, eps_rel=1e-3)
        res = prob.solve()
        if res.info.status != 'solved':
            return self.on_solve_failure(v_last_cmd)
        return res.x[0]

    def on_solve_failure(self, v_last_cmd):
        return v_last_cmd

    def control_loop(self, event):
        if not self.can_move:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0

        self._check_lock()

        if self.is_stopped:
            self.sport_client.Move(0.0, 0.0, 0.0)
            self.last_cmd_vx = 0.0
            self.last_cmd_vy = 0.0
            self.last_cmd_wz = 0.0
            return

        mode = self.get_adaptive_mode()
        if mode != self.last_mode:
            self.mode_switch_count += 1
            mode_name = "高速巡航" if mode == "fast" else "低速精准"
            rospy.loginfo(f"🔄 模式切换: {mode_name} (第{self.mode_switch_count}次)")
            self.last_mode = mode

        acc_w = self.max_acc_w_fast if mode == "fast" else self.max_acc_w_slow
        R_ang = self.R_v_ang_fast if mode == "fast" else self.R_v_ang_slow

        cmd_vx = self.solve_step(self.current_vx, self.target_vx, self.last_cmd_vx,
                                 self.max_vx, self.max_acc_v, R_v=self.R_v_lin)
        cmd_vy = self.solve_step(self.current_vy, self.target_vy, self.last_cmd_vy,
                                 self.max_vy, self.max_acc_v, R_v=self.R_v_lin)
        cmd_wz = self.solve_step(self.current_wz, self.target_wz, self.last_cmd_wz,
                                 self.max_wz, acc_w, R_v=R_ang)

        if int(rospy.get_time() * 5) % 1 == 0:
            mode_tag = "F" if mode == "fast" else "S"
            print(f"[{mode_tag}] Target: ({self.target_vx:.2f}, {self.target_wz:.2f}) | "
                  f"Current: ({self.current_vx:.2f}, {self.current_wz:.2f}) -> "
                  f"Send: ({cmd_vx:.2f}, {cmd_wz:.2f})")

        if abs(cmd_vx) < 0.01 and abs(cmd_vy) < 0.01 and abs(cmd_wz) < 0.01:
            self.sport_client.StopMove()
        else:
            self.sport_client.Move(cmd_vx, cmd_vy, cmd_wz)

        self.last_cmd_vx = cmd_vx
        self.last_cmd_vy = cmd_vy
        self.last_cmd_wz = cmd_wz


if __name__ == "__main__":
    G1BaseController.run(AdaptiveMPCController)
