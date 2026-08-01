#!/usr/bin/env python3
"""G1 MPC 控制器（锁定停止模式）。基类 G1BaseController + OSQP 单步求解。"""
import rospy
import numpy as np
import osqp
import scipy.sparse as sp
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

from g1_controller_base import G1BaseController


class MPCController(G1BaseController):
    node_name = "unitree_mpc_controller"

    def configure_params(self):
        self.max_vx = 1.5
        self.max_vy = 0.6
        self.max_wz = 1.5
        self.max_acc_v = 5.0
        self.max_acc_w = 5.0
        self.Q_v = 10.0
        self.R_v = 2.0
        self.stop_velocity_threshold = 0.03

    def setup_feedback(self):
        self.odom_dds_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
        self.odom_dds_sub.Init(self.dds_odom_callback)

    def dds_odom_callback(self, msg: SportModeState_):
        self.current_vx = msg.velocity[0]
        self.current_vy = msg.velocity[1]
        self.current_wz = msg.yaw_speed

    def solve_step(self, v_current, v_target, v_last_cmd, max_v, max_acc, **kw):
        v_current = np.clip(v_current, -max_v, max_v)
        P = sp.csc_matrix([[2 * (self.Q_v + self.R_v)]])
        q = np.array([-2 * (self.Q_v * v_target + self.R_v * v_last_cmd)])
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


if __name__ == "__main__":
    G1BaseController.run(MPCController)
