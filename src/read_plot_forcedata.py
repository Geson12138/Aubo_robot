import socket
import json
import struct
import time
import threading
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

# 定义报文头和尾
PACK_BEGIN = "<PACK_BEGIN"
PACK_END = "PACK_END>"

# 全局变量用于存储力传感数据和时间戳，最大长度为4000
force_data_list = deque(maxlen=4000)
time_data_list = deque(maxlen=4000)

# 全局变量用于控制程序运行
running = True

def read_message(sock):
    global force_data_list, time_data_list, running
    buffer = b""
    
    while running:
        data = sock.recv(4096)
        if not data:
            break
        buffer += data
        
        begin_pos = buffer.find(PACK_BEGIN.encode())
        end_pos = buffer.find(PACK_END.encode())
        
        if begin_pos >= 0 and end_pos >= 0 and end_pos > begin_pos + len(PACK_BEGIN.encode()):
            packet = buffer[begin_pos:end_pos + len(PACK_END.encode())]
            packet_content = packet[len(PACK_BEGIN.encode()):-len(PACK_END.encode())]
            length_str = packet_content[:8].decode().strip()
            length = int(length_str)
            json_data = packet_content[8:8+length].decode()
            
            try:
                json_obj = json.loads(json_data)
                force_data = json_obj["force_data"]
                current_time = time.time()
                
                force_data_list.append(force_data)
                time_data_list.append(current_time)
                
                buffer = buffer[end_pos + len(PACK_END.encode()):]
                
            except json.JSONDecodeError as e:
                print("JSON解析错误:", e)
                buffer = buffer[end_pos + len(PACK_END.encode()):]
                continue

def on_close(event):
    global running
    running = False
    plt.close(event.canvas.figure)

def main():
    
    global running
    server_address = ('192.168.26.103', 8896)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(server_address)
    
    read_thread = threading.Thread(target=read_message, args=(sock,))
    read_thread.start()
    
    plt.ion()  # 开启交互模式
    fig, (ax1, ax2) = plt.subplots(2, 1)
    fig.canvas.mpl_connect('close_event', on_close)
    
    lines1 = [ax1.plot([], [], label=label, color=color)[0] for label, color in zip(['fx', 'fy', 'fz'], ['r', 'g', 'b'])]
    lines2 = [ax2.plot([], [], label=label, color=color)[0] for label, color in zip(['mx', 'my', 'mz'], ['r', 'g', 'b'])]
    
    ax1.set_xlim(0, 20)  # 显示最近20秒的数据
    ax1.set_ylim(-10, 10)
    ax1.legend()
    
    ax2.set_xlim(0, 20)  # 显示最近20秒的数据
    ax2.set_ylim(-10, 10)
    ax2.legend()
    
    while running:
        if len(force_data_list) == len(time_data_list) and force_data_list:
            data = np.array(force_data_list)  # 转换为numpy数组
            times = np.array(time_data_list) - time_data_list[0]  # 转换为相对时间
            
            if len(times) == data.shape[0]:
                for i, line in enumerate(lines1):
                    line.set_xdata(times)
                    line.set_ydata(data[:, i])
                
                for i, line in enumerate(lines2):
                    line.set_xdata(times)
                    line.set_ydata(data[:, i + 3])
                
                ax1.set_xlim(0, times[-1])
                ax1.set_ylim(np.min(data[:, :3]) - 1, np.max(data[:, :3]) + 1)
                
                ax2.set_xlim(0, times[-1])
                ax2.set_ylim(np.min(data[:, 3:]) - 1, np.max(data[:, 3:]) + 1)
                
                plt.draw()
                plt.pause(0.01)
    
    read_thread.join()
    sock.close()

if __name__ == "__main__":
    main()