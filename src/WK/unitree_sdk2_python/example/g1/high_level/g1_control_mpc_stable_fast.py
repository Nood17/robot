#!/usr/bin/env python3
"""G1 自适应 MPC 控制器（Bidirectional，允许倒车，全向移动）。

继承 G1BaseController，覆盖：
  - configure_params: Q_v=8（唯一），角轴权重 5/4，abs 模式
  - process_cmd_vel: 允许倒车；角速度限幅 1.0/(|vx|+1.8) [0.2,0.7]
  - solve_step: 角轴特殊处理（decel_threshold=0.20，approaching_zero×0.7）
  - get_adaptive_mode: 基于 abs(target_vx)
  - control_loop: 模式切换 + 调试打印
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
        self.max_acc_w_fast = 3.0
        self.max_acc_w_slow = 2.5
        self.R_v_lin = 1.0
        self.R_v_ang_fast = 5
        self.R_v_ang_slow = 4.0
        self.Q_v = 8.0
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
        enter_fast_threshold = 0.4
        exit_fast_threshold = 0.3
        abs_target = abs(self.target_vx)
        if self.last_mode == "fast":
            if abs_target < exit_fast_threshold:
                return "slow"
            else:
                return "fast"
        else:
            if abs_target > enter_fast_threshold:
                return "fast"
            else:
                return "slow"

    def process_cmd_vel(self, msg):
        # 允许倒车，不再拦截负速度
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        mode = self.get_adaptive_mode()
        raw_wz = msg.angular.z
        if mode == "fast":
            if abs(raw_wz) > 0.05:
                max_allowed_wz = 1.0 / (abs(self.current_vx) + 1.8)
                max_allowed_wz = max(0.2, min(0.7, max_allowed_wz))
                self.target_wz = np.clip(raw_wz, -max_allowed_wz, max_allowed_wz)
            else:
                self.target_wz = 0.0
        else:
            self.target_wz = raw_wz

    def solve_step(self, v_current, v_target, v_last_cmd, max_v, max_acc, **kw):
        mode = kw.get("mode", "slow")
        is_angular = kw.get("is_angular", False)
        # 1. 输入截断
        v_current = np.clip(v_current, -max_v * 1.2, max_v * 1.2)
        v_target = np.clip(v_target, -max_v, max_v)
        # 2. 权重选择
        if is_angular:
            R_v = self.R_v_ang_fast if mode == "fast" else self.R_v_ang_slow
        else:
            R_v = self.R_v_lin
        P = sp.csc_matrix([[2 * (self.Q_v + R_v)]])
        q = np.array([-2 * (self.Q_v * v_target + R_v * v_last_cmd)])
        acc_limit = max_acc * self.dt
        # 3. 角速度特殊处理
        if is_angular:
            decel_threshold = 0.20  # 0.05 -> 0.15
            same_direction = (v_current * v_target >= 0)
            need_decel = abs(v_current) > abs(v_target) + decel_threshold
            approaching_zero = abs(v_target) < 0.1 and abs(v_current) > 0.1
            if approaching_zero:
                v_base = v_current * 0.7  # 提前刹车
            elif same_direction and need_decel:
                v_base = v_current
            else:
                v_base = 0.6 * v_current + 0.4 * v_last_cmd
        else:
            same_direction = (v_current * v_target >= 0)
            need_decel = abs(v_current) > abs(v_target) + 0.05
            if mode == "fast":
                if same_direction and need_decel:
                    v_base = v_current
                else:
                    v_base = 0.6 * v_current + 0.4 * v_last_cmd
            else:
                if same_direction and need_decel:
                    v_base = v_current
                else:
                    v_base = 0.7 * v_current + 0.3 * v_last_cmd
        # 4. 边界
        lower_val = max(-max_v, v_base - acc_limit)
        upper_val = min(max_v, v_base + acc_limit)
        if lower_val > upper_val:
            lower_val = -max_v
            upper_val = max_v
        A_box = sp.csc_matrix([[1.0]])
        prob = osqp.OSQP()
        prob.setup(P, q, A_box, np.array([lower_val]), np.array([upper_val]),
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
            strategy = "加权(快速)" if mode == "fast" else "混合(精准)"
            rospy.loginfo(f"🔄 模式切换: {mode} ({strategy})")
            self.last_mode = mode

        acc_w = self.max_acc_w_fast if mode == "fast" else self.max_acc_w_slow

        cmd_vx = self.solve_step(self.current_vx, self.target_vx, self.last_cmd_vx,
                                 self.max_vx, self.max_acc_v, mode=mode)
        cmd_vy = self.solve_step(self.current_vy, self.target_vy, self.last_cmd_vy,
                                 self.max_vy, self.max_acc_v, mode=mode)
        cmd_wz = self.solve_step(self.current_wz, self.target_wz, self.last_cmd_wz,
                                 self.max_wz, acc_w, mode=mode, is_angular=True)

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
