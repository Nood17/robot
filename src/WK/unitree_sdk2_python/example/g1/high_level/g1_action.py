#!/usr/bin/env python3
"""G1 动作控制器（基础版）。无限位，显式时长，time.sleep 保持。

继承 G1ActionController，只提供 peace_poses；其余用基类默认。
"""
import math
from g1_action_base import G1ActionController as _Base, JointIndex


class G1ActionController(_Base):
    def get_peace_poses(self, half_pi):
        return {
            JointIndex.LeftShoulderPitch: 0.0,
            JointIndex.LeftShoulderRoll: -0.50,
            JointIndex.LeftShoulderYaw: 0.0,
            JointIndex.LeftElbowPitch: half_pi,
            JointIndex.LeftElbowRoll: 0.0,
            JointIndex.RightShoulderPitch: -half_pi,
            JointIndex.RightShoulderRoll: -half_pi,
            JointIndex.RightShoulderYaw: 0.0,
            JointIndex.RightElbowPitch: -half_pi,    # 肘部微弯
            JointIndex.RightElbowRoll: 1.0,    # 手腕旋转
            JointIndex.WaistYaw: 0.0,
            JointIndex.WaistRoll: 0.0,
            JointIndex.WaistPitch: 0.0,
        }


if __name__ == "__main__":
    from g1_action_base import main
    main(G1ActionController)
