import json
import numpy as np
import matplotlib.pyplot as plt

# 读取 trajectory_data.json 文件
with open('trajectory_data.json', 'r') as f:
    trajectory_data = json.load(f)

with open('trajectory_data_real.json', 'r') as f:
    trajectory_data_real = json.load(f)

# 将数据转换为 NumPy 数组
trajectory_array = np.array([np.array(item) for item in trajectory_data])  # 形状 (n_samples, 6)
print(trajectory_array.shape[0])

# 将数据转换为 NumPy 数组
trajectory_array_real = np.array([np.array(item) for item in trajectory_data_real])  # 形状 (n_samples, 6)
print(trajectory_array_real.shape[0])

# 获取样本数量
n_samples = trajectory_array_real.shape[0]

# 创建一个窗口，包含6个子图
fig, axes = plt.subplots(6, 1, figsize=(12, 18))
joint_labels = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']

# 绘制每个关节的变化曲线
for i in range(6):
    axes[i].plot(np.arange(n_samples), trajectory_array[:n_samples, i], label=joint_labels[i], color='blue')
    axes[i].plot(np.arange(n_samples), trajectory_array_real[:, i], label=joint_labels[i], color='red')
    axes[i].set_xlabel('Sample')
    axes[i].set_ylabel('Angle (rad)')
    axes[i].legend()
    axes[i].grid(True)

plt.tight_layout()
plt.show()