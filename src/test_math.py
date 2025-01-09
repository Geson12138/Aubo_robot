import numpy as np


control_period = 0.04  # 控制周期 (s)
acc_time = 1 * control_period  # 加速时间 (s)
dcc_time = 1 * control_period  # 减速时间 (s)
joint_vel_limit = np.ones(6) * 0.04  # 每个关节的最大角速度 (rad/s)
joint_acc_limit = np.ones(6) * 0.08  # 计算最大角加速度

prev_vel =  np.zeros(6) # 保存上一周期周期的最大关节速度
# Calculate velocity bounds considering direction
vel_lower_bound = prev_vel - joint_acc_limit * control_period
vel_upper_bound = prev_vel + joint_acc_limit * control_period


# 打印变量的值和类型
print(f"control_period: {control_period}, type: {type(control_period)}")
print(f"acc_time: {acc_time}, type: {type(acc_time)}")
print(f"dcc_time: {dcc_time}, type: {type(dcc_time)}")
print(f"joint_vel_limit: {joint_vel_limit}, type: {type(joint_vel_limit)}")
print(f"joint_acc_limit: {joint_acc_limit}, type: {type(joint_acc_limit)}")

print(f"prev_vel: {prev_vel}, type: {type(prev_vel)}")
print(f"vel_lower_bound: {vel_lower_bound}, type: {type(vel_lower_bound)}")
print(f"vel_upper_bound: {vel_upper_bound}, type: {type(vel_upper_bound)}")