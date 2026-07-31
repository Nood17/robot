#!/usr/bin/env python3
"""G1 自适应 MPC 控制器（v9 Final Optimized）。

继承 G1BaseController，覆盖：
  - configure_params: 分轴加速度、模式权重、tolerance 基准裁剪
  - setup_feedback 后追加 SDK 接管（Start + SwitchMoveMode）
  - process_cmd_vel: 快速模式角速度限幅 [0.2, 0.6]
  - solve_step: 容差基准裁剪 + R_weight 参数
  - control_loop: 自适应模式切换 + 调试打印
注意：本变体 can_move 初值为 True（唯一）。
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
    can_move_initial = True

    def configure_params(self):
        self.max_vx = 1.5
        self.max_vy = 0.6
        self.max_wz = 1.5
        # Fast 模式参数 (调整: 提升转向响应)
        self.max_acc_w_fast = 3.5   # 3.0 -> 3.5
        self.R_v_ang_fast = 2.5     # 3.0 -> 2.5 (稍微降低平滑度，提升响应)
        # Slow 模式参数
        self.max_acc_w_slow = 4.0
        self.R_v_ang_slow = 2.0
        self.max_acc_v = 5.0
        self.Q_v = 10.0
        self.R_v_lin = 2.0
        self.adaptive_mode = "auto"
        self.deadzone_threshold = 0.05
        self.last_mode = "slow"
        self.mode_switch_count = 0
        self.stop_velocity_threshold = 0.03

    def setup_feedback(self):
        # SDK 接管：给底层建立连接时间，切入主运控，开启 Move 持续响应
        rospy.sleep(0.5)
        self.sport_client.Start()
        try:
            self.sport_client.SwitchMoveMode(True)
        except AttributeError:
            pass  # SDK 版本太老无此函数则跳过
        # DDS 里程计
        self.odom_dds_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
        self.odom_dds_sub.Init(self.dds_odom_callback)

    def dds_odom_callback(self, msg: SportModeState_):
        self.current_vx = msg.velocity[0]
        self.current_vy = msg.velocity[1]
        self.current_wz = msg.yaw_speed

    def get_adaptive_mode(self):
        if self.adaptive_mode != "auto":
            return self.adaptive_mode
        # 【修改】调整阈值，让 Fast 模式更"粘"
        enter_fast_threshold = 0.4
        exit_fast_threshold = 0.15  # 大幅降低，防止减速时过早切 Slow
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
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        mode = self.get_adaptive_mode()
        raw_wz = msg.angular.z
        if mode == "fast":
            if abs(raw_wz) < self.deadzone_threshold:
                self.target_wz = 0.0
            else:
                if self.current_vx > 0.2:
                    # 【修改】放宽角速度公式：0.8 -> 1.0，下限 0.6
                    max_allowed_wz = 1.0 / (self.current_vx + 1.0)
                    max_allowed_wz = max(0.2, min(0.6, max_allowed_wz))
                    self.target_wz = np.clip(raw_wz, -max_allowed_wz, max_allowed_wz)
                else:
                    self.target_wz = raw_wz
        else:
            self.target_wz = raw_wz

    def solve_step(self, v_current, v_target, v_last_cmd, max_v, max_acc, **kw):
        R_weight = kw.get("R_v", self.R_v_lin)
        v_current = np.clip(v_current, -max_v, max_v)
        P = sp.csc_matrix([[2 * (self.Q_v + R_weight)]])
        q = np.array([-2 * (self.Q_v * v_target + R_weight * v_last_cmd)])
        acc_limit = max_acc * self.dt
        lower_bound = np.array([max(-max_v, v_current - acc_limit)])
        upper_bound = np.array([min(max_v, v_current + acc_limit)])
        A_box = sp.csc_matrix([[1.0]])
        prob = osqp.OSQP()
        prob.setup(P, q, A_box, lower_bound, upper_bound, verbose=False, eps_abs=1e-3, eps_rel=1e-3)
        res = prob.solve()
        if res.info.status != 'solved':
            return self.on_solve_failure(v_last_cmd)
        return res.x[0]

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

        if mode == "fast":
            R_ang = self.R_v_ang_fast
            acc_w = self.max_acc_w_fast
        else:
            R_ang = self.R_v_ang_slow
            acc_w = self.max_acc_w_slow

        # tolerance 基准裁剪：限制 last_cmd 相对 current 的偏差
        tol_lin = 0.2  # 线速度最大允许偏差 (m/s)
        tol_ang = 0.3  # 角速度最大允许偏差 (rad/s)
        base_vx = np.clip(self.last_cmd_vx, self.current_vx - tol_lin, self.current_vx + tol_lin)
        base_vy = np.clip(self.last_cmd_vy, self.current_vy - tol_lin, self.current_vy + tol_lin)
        base_wz = np.clip(self.last_cmd_wz, self.current_wz - tol_ang, self.current_wz + tol_ang)

        cmd_vx = self.solve_step(base_vx, self.target_vx, self.last_cmd_vx, self.max_vx, self.max_acc_v, R_v=self.R_v_lin)
        cmd_vy = self.solve_step(base_vy, self.target_vy, self.last_cmd_vy, self.max_vy, self.max_acc_v, R_v=self.R_v_lin)
        cmd_wz = self.solve_step(base_wz, self.target_wz, self.last_cmd_wz, self.max_wz, acc_w, R_v=R_ang)

        if int(rospy.get_time() * 5) % 10 == 0:
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
