#!/usr/bin/env python3
"""G1 开环控制器（EMA 平滑 + 计数器到位锁定）。

无里程计反馈，current_* 始终为 0；EMA 滤波器用独立的 fstate_* 维护。
锁定机制为 counter 模式（锁定 100 周期 ≈ 2 秒）。
"""
import rospy
from g1_controller_base import G1BaseController


class CmdVelController(G1BaseController):
    node_name = "unitree_cmd_vel_controller"
    lock_style = "counter"
    use_stopmove_deadband = False   # 开环总是 Move()

    def configure_params(self):
        # 【核心修改1】超软刹车系数，防止急刹触发平衡
        self.smoothing_factor_v = 0.2  # 从1改成0.05，刹车时间延长到约2秒
        self.smoothing_factor_w = 0.4   # 角速度保持较快
        self.stop_threshold_cycles = 100  # 50Hz * 2秒 = 锁定2秒
        self.stop_velocity_threshold = 0.02  # 停止阈值
        # EMA 滤波器内部状态（与 current_* 反馈解耦）
        self._fstate_vx = 0.0
        self._fstate_vy = 0.0
        self._fstate_wz = 0.0
        # max_v / max_acc 开环不使用，占位以满足基类 control_loop 调用
        self.max_vx = 1e9
        self.max_vy = 1e9
        self.max_wz = 1e9
        self.max_acc_v = 0.0
        self.max_acc_w = 0.0

    def setup_feedback(self):
        # 开环无反馈源
        pass

    def solve_step(self, v_current, v_target, v_last_cmd, max_v, max_acc, **kw):
        # v_current 在开环中恒为 0（无反馈），EMA 用 _fstate 独立维护
        # 此处 axis 通过 kw 传入以选择对应滤波状态
        axis = kw.get("axis", "vx")
        if axis == "vx":
            f = self.smoothing_factor_v
            self._fstate_vx = (1 - f) * self._fstate_vx + f * v_target
            return self._fstate_vx
        elif axis == "vy":
            f = self.smoothing_factor_v
            self._fstate_vy = (1 - f) * self._fstate_vy + f * v_target
            return self._fstate_vy
        else:  # wz
            f = self.smoothing_factor_w
            self._fstate_wz = (1 - f) * self._fstate_wz + f * v_target
            return self._fstate_wz

    def control_loop(self, event):
        """覆盖基类循环：开环用 axis 参数调用 solve_step。"""
        if not self.can_move:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0

        self._check_lock()

        if self.is_stopped:
            if self.stop_counter > 0:
                self.sport_client.Move(0.0, 0.0, 0.0)
                self.stop_counter -= 1
            else:
                self.is_stopped = False
            return

        cmd_vx = self.solve_step(0.0, self.target_vx, self._fstate_vx, self.max_vx, self.max_acc_v, axis="vx")
        cmd_vy = self.solve_step(0.0, self.target_vy, self._fstate_vy, self.max_vy, self.max_acc_v, axis="vy")
        cmd_wz = self.solve_step(0.0, self.target_wz, self._fstate_wz, self.max_wz, self.max_acc_w, axis="wz")

        self.sport_client.Move(cmd_vx, cmd_vy, cmd_wz)
        self.last_cmd_vx = cmd_vx
        self.last_cmd_vy = cmd_vy
        self.last_cmd_wz = cmd_wz


if __name__ == "__main__":
    rospy.init_node("unitree_cmd_vel_controller", anonymous=False)
    rospy.logwarn("⚠️ 已启用中等平滑(0.2) + 2秒到位锁定 + 分段速度控制")
    G1BaseController.run(CmdVelController)
