import json
import numpy as np
import matplotlib.pyplot as plt

# 读取 trajectory_data.json 文件
with open('trajectory_data.json', 'r') as f:
    trajectory_data = json.load(f)

with open('trajectory_data_real.json', 'r') as f:
    trajectory_data_real = json.load(f)

with open('trajectory_velocity_real.json', 'r') as f:
    trajectory_velocity_real = json.load(f)

# 将数据转换为 NumPy 数组
trajectory_array = np.array([np.array(item) for item in trajectory_data])  # 形状 (n_samples, 6)
print(trajectory_array.shape[0])

# 将数据转换为 NumPy 数组
trajectory_array_real = np.array([np.array(item) for item in trajectory_data_real])  # 形状 (n_samples, 6)
print(trajectory_array_real.shape[0])

# 将数据转换为 NumPy 数组
trajectory_velocity_real = np.array([np.array(item) for item in trajectory_velocity_real])  # 形状 (n_samples, 6)
print(trajectory_velocity_real.shape[0])

# ==============================实际关节速度曲线==============================
# trajectory_velocity_real = np.diff(trajectory_array_real, axis=0) / 0.04
n_samples = trajectory_velocity_real.shape[0]

# 创建一个窗口，包含6个子图
fig, axes = plt.subplots(6, 1, figsize=(12, 18))
joint_labels = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']

# 绘制每个关节的变化曲线
for i in range(6):
    axes[i].plot(np.arange(n_samples), trajectory_velocity_real[:, i], label=f'Velocities {joint_labels[i]}', color='green')
    axes[i].set_xlabel('Sample')
    axes[i].set_ylabel('Angle (rad)')
    axes[i].legend()
    axes[i].grid(True)
plt.tight_layout()
plt.savefig('trajectory_velocity_real.png')
# ==============================实际关节速度曲线==============================

# ==============================实际关节加速度曲线==============================
trajectory_acc_real = np.diff(trajectory_velocity_real, axis=0) / 0.04
n_samples = trajectory_acc_real.shape[0]
print(trajectory_acc_real.shape[0])

# 创建一个窗口，包含6个子图
fig, axes = plt.subplots(6, 1, figsize=(12, 18))
joint_labels = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']

# 绘制每个关节的变化曲线
for i in range(6):
    axes[i].plot(np.arange(n_samples), trajectory_acc_real[:, i], label=f'acceleration {joint_labels[i]}', color='orange')
    axes[i].set_xlabel('Sample')
    axes[i].set_ylabel('Angle (rad)')
    axes[i].legend()
    axes[i].grid(True)
plt.tight_layout()
plt.savefig('trajectory_acc_real.png')
# ==============================实际关节速度曲线==============================

# ==============================实际关节变化曲线==============================
# 获取样本数量
n_samples = trajectory_array_real.shape[0]

# 创建一个窗口，包含6个子图
fig, axes = plt.subplots(6, 1, figsize=(12, 18))
joint_labels = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']

# 绘制每个关节的变化曲线
for i in range(6):
    axes[i].plot(np.arange(n_samples), trajectory_array_real[:, i], label=f'Real {joint_labels[i]}', color='red')
    # axes[i].plot(np.arange(n_samples), trajectory_array[:, i], label=f'Desired {joint_labels[i]}' ,color='blue')
    axes[i].set_xlabel('Sample')
    axes[i].set_ylabel('Angle (rad)')
    axes[i].legend()
    axes[i].grid(True)
plt.tight_layout()
plt.savefig('trajectory_array_real.png')
# ==============================实际关节变化曲线==============================

# ==============================期望关节变化曲线==============================
# 创建一个窗口，包含6个子图
fig, axes = plt.subplots(6, 1, figsize=(12, 18))
joint_labels = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']

# 获取样本数量
n_samples = trajectory_array.shape[0]

# 绘制每个关节的变化曲线
for i in range(6):
    axes[i].plot(np.arange(n_samples), trajectory_array[:, i], label=f'Desired {joint_labels[i]}' ,color='blue')
    axes[i].set_xlabel('Sample')
    axes[i].set_ylabel('Angle (rad)')
    axes[i].legend()
    axes[i].grid(True)
plt.tight_layout()
plt.savefig('trajectory_array_desired.png')
# ==============================期望关节变化曲线==============================


plt.show()