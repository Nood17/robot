#!/usr/bin/env python3
"""G1 PID 控制器。ROS /odom 反馈，三轴独立 PID，无锁定机制。"""
import rospy
from nav_msgs.msg import Odometry
from g1_controller_base import G1BaseController


class PIDController(G1BaseController):
    node_name = "unitree_pid_controller"
    lock_style = "none"
    use_stopmove_deadband = False   # PID 总是 Move()

    def configure_params(self):
        # PID 参数 (三个方向独立控制)
        self.kp_vx = 2.0    # 比例系数：响应速度
        self.ki_vx = 0.1    # 积分系数：消除稳态误差
        self.kd_vx = 0.05   # 微分系数：抑制震荡
        self.kp_vy = 2.0
        self.ki_vy = 0.1
        self.kd_vy = 0.05
        self.kp_wz = 2.5
        self.ki_wz = 0.15
        self.kd_wz = 0.08
        # 速度限幅 (安全约束)
        self.max_vx = 0.8
        self.max_vy = 0.4
        self.max_wz = 1.0
        # 积分限幅 (防止积分饱和)
        self.max_integral_v = 0.5
        self.max_integral_w = 0.8
        # max_acc 占位（基类 control_loop 引用，PID 不使用）
        self.max_acc_v = 0.0
        self.max_acc_w = 0.0
        # PID 状态
        self.last_error_vx = 0.0
        self.last_error_vy = 0.0
        self.last_error_wz = 0.0
        self.integral_vx = 0.0
        self.integral_vy = 0.0
        self.integral_wz = 0.0

    def setup_feedback(self):
        # 订阅里程计，获取真实速度反馈
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)

    def odom_callback(self, msg: Odometry):
        self.current_vx = msg.twist.twist.linear.x
        self.current_vy = msg.twist.twist.linear.y
        self.current_wz = msg.twist.twist.angular.z

    def pid_compute(self, target, current, last_error, integral, kp, ki, kd, max_integral):
        error = target - current
        p_term = kp * error
        integral = integral + error * self.dt
        if integral > max_integral:
            integral = max_integral
        elif integral < -max_integral:
            integral = -max_integral
        i_term = ki * integral
        derivative = (error - last_error) / self.dt
        d_term = kd * derivative
        output = p_term + i_term + d_term
        return output, error, integral

    def solve_step(self, v_current, v_target, v_last_cmd, max_v, max_acc, **kw):
        axis = kw.get("axis")
        if axis == "vx":
            out, err, self.integral_vx = self.pid_compute(
                v_target, v_current, self.last_error_vx, self.integral_vx,
                self.kp_vx, self.ki_vx, self.kd_vx, self.max_integral_v)
            self.last_error_vx = err
        elif axis == "vy":
            out, err, self.integral_vy = self.pid_compute(
                v_target, v_current, self.last_error_vy, self.integral_vy,
                self.kp_vy, self.ki_vy, self.kd_vy, self.max_integral_v)
            self.last_error_vy = err
        else:  # wz
            out, err, self.integral_wz = self.pid_compute(
                v_target, v_current, self.last_error_wz, self.integral_wz,
                self.kp_wz, self.ki_wz, self.kd_wz, self.max_integral_w)
            self.last_error_wz = err
        return max(min(out, max_v), -max_v)

    def control_loop(self, event):
        if not self.can_move:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wz = 0.0
        # PID 无锁定，直接求解
        cmd_vx = self.solve_step(self.current_vx, self.target_vx, self.last_cmd_vx,
                                 self.max_vx, self.max_acc_v, axis="vx")
        cmd_vy = self.solve_step(self.current_vy, self.target_vy, self.last_cmd_vy,
                                 self.max_vy, self.max_acc_v, axis="vy")
        cmd_wz = self.solve_step(self.current_wz, self.target_wz, self.last_cmd_wz,
                                 self.max_wz, self.max_acc_w, axis="wz")
        self.sport_client.Move(cmd_vx, cmd_vy, cmd_wz)
        self.last_cmd_vx = cmd_vx
        self.last_cmd_vy = cmd_vy
        self.last_cmd_wz = cmd_wz


if __name__ == "__main__":
    rospy.init_node("unitree_pid_controller", anonymous=False)
    rospy.logwarn("🎮 PID 控制器已启动")
    rospy.loginfo("参数: Kp=2.0, Ki=0.1, Kd=0.05 (线速度)")
    G1BaseController.run(PIDController)
