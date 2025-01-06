import roboticstoolbox as rtb
import numpy as np

# --- DH参数 (仅基于上表的近似推导) ---
#   alpha, a, d 都是按“标准DH”定义
#   offset=0 表示没有附加零位偏置

L1 = rtb.RevoluteDH(alpha=0,              a=0,     d=0.197,    offset=0)
L2 = rtb.RevoluteDH(alpha=-np.pi/2,       a=647,   d=0,      offset=0)
L3 = rtb.RevoluteDH(alpha=0,              a=600.5, d=0,      offset=0)
L4 = rtb.RevoluteDH(alpha=-np.pi/2,       a=0,     d=0.1235,  offset=0)
L5 = rtb.RevoluteDH(alpha=np.pi/2,        a=0,     d=0.1278,  offset=0)
L6 = rtb.RevoluteDH(alpha=-np.pi/2,       a=0,     d=0.094,     offset=0)

# 组合成一个6关节机器人
aubo_i10 = rtb.DHRobot(
    [L1, L2, L3, L4, L5, L6],
    name='AUBO_i10_DH'
)

print(aubo_i10)

# 测试：给定一组关节角q(单位:弧度)
q_test = [1.3905, -0.085, -1.7465, 1.5111, 1.3983, 3.1527]

# 正运动学
T_end = aubo_i10.fkine(q_test)
print("End-effector pose at q_test = \n", T_end)

# 若需要逆运动学
sol = aubo_i10.ikine_LM(T_end)
print("IK solution = ", sol.q)
print("Check FK of IK solution = \n", aubo_i10.fkine(sol.q))
