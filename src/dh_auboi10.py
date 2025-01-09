import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteMDH
import pykin.utils.transform_utils as transform_utils

deg2rad = np.pi / 180.0
scale_factor = 1000

class DH_Auboi10():
        
    def __init__(self):

        # 假设这六个关节的 MDH 参数 (alpha_i, a_i, d_i, offset)
        self.joint1 = RevoluteMDH(
            alpha  =   0.0 * deg2rad,    # alpha_1
            a      =   0.0 / scale_factor,               # a_1
            d      = 163.2 / scale_factor,               # d_1
            offset = 180.0 * deg2rad      # 零位偏移(°->rad)
        )

        self.joint2 = RevoluteMDH(
            alpha  =  -90.0 * deg2rad,
            a      =   0.0 / scale_factor,
            d      =  0.0 / scale_factor,
            offset =  -90.0 * deg2rad
        )

        self.joint3 = RevoluteMDH(
            alpha  =  180.0 * deg2rad,
            a      =  647.0 / scale_factor,
            d      =   0.0 / scale_factor,
            offset =   0.0 * deg2rad
        )

        self.joint4 = RevoluteMDH(
            alpha  =  180.0 * deg2rad,
            a      =  600.5 / scale_factor,
            d      =  201.3 / scale_factor,
            offset =  -90.0 * deg2rad
        )

        self.joint5 = RevoluteMDH(
            alpha  =  -90.0 * deg2rad,
            a      =    0.0 / scale_factor,
            d      =  102.5 / scale_factor,
            offset =    0.0 * deg2rad
        )

        self.joint6 = RevoluteMDH(
            alpha  =   90.0 * deg2rad,
            a      =    0.0 / scale_factor,
            d      =   94.0 / scale_factor,
            offset =    0.0 * deg2rad
        )

        # 组合为一个 DHRobot
        self.i10_robot_mdh = DHRobot(
            [self.joint1, self.joint2, self.joint3, self.joint4, self.joint5, self.joint6],
            name='i10_robot_MDH'
        )

    def dh_fkine(self, q):
        
        T_end = self.i10_robot_mdh.fkine(q)
        ee_pose = T_end.A
        return ee_pose

    def jacob0(self, q):
        J = self.i10_robot_mdh.jacob0(q)
        return J
    
if __name__ == "__main__":

    # 创建机器人对象
    i10_robot_mdh = DH_Auboi10()

    # 测试正向运动学
    q_test = [1.2384, -0.0699, -1.7395, 1.4956, 1.2689, 3.1505]
    T_end = i10_robot_mdh.dh_fkine(q_test)
    print(type(T_end))
    print("当关节变量都=0时, 末端位姿:\n", T_end)
    print('末端位置: ', T_end[:3, 3])
    print('末端姿态: ', transform_utils.get_rpy_from_matrix(T_end[:3, :3]))

    # 测试雅可比矩阵
    J = i10_robot_mdh.jacob0(q_test)
    print("雅可比矩阵:\n", J)

    # 可视化 (matplotlib 3D)
    # i10_robot_mdh.plot(q_test, block=True)

