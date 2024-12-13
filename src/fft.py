import json
import numpy as np
import matplotlib.pyplot as plt

# 设置采样参数
dt = 0.005  # 采样间隔 5ms
fs = 1 / dt  # 采样频率 200Hz

# 读取 force_data.json 文件
with open('force_data.json', 'r') as f:
    force_data_list = json.load(f)

# 将数据转换为 NumPy 数组
force_data_array = np.array(force_data_list)  # 形状 (n_samples, 6)

# 获取样本数量
n_samples = force_data_array.shape[0]

# 对每一列的 force_data 进行傅里叶变换
fft_results = np.fft.fft(force_data_array, axis=0)
fft_freqs = np.fft.fftfreq(n_samples, d=dt)

# 计算幅值谱
amplitude_spectrum = np.abs(fft_results) / n_samples

# 只取正频率部分
positive_freq_indices = np.where(fft_freqs >= 0)
fft_freqs = fft_freqs[positive_freq_indices]
amplitude_spectrum = amplitude_spectrum[positive_freq_indices]

# 绘制频率和幅值的曲线图
fig, axes = plt.subplots(6, 1, figsize=(12, 18))
labels = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']

for i in range(6):
    axes[i].plot(fft_freqs, amplitude_spectrum[:, i], label=f'{labels[i]} Amplitude')
    axes[i].set_xlabel('Frequency (Hz)')
    axes[i].set_ylabel('Amplitude')
    axes[i].legend()
    axes[i].grid(True)

plt.tight_layout()
plt.show()