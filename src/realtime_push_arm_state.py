import socket
import time
import numpy as np
import logging
from logging.handlers import RotatingFileHandler
import os
import matplotlib.pyplot as plt
import time
import socket
import threading
import json
from lib.robotcontrol import Auboi5Robot, RobotError, RobotErrorType
from pykin.robots.single_arm import SingleArm
from pykin.utils import transform_utils as transform_utils
from spatialmath import SE3
from scipy.signal import butter, filtfilt
import queue
import concurrent.futures
from collections import deque

# 创建一个logger
logger = logging.getLogger('calibration_forceSensor')

# 清除上一次的logger
if logger.hasHandlers():
    logger.handlers.clear()

def logger_init():
    
    logger.setLevel(logging.INFO) # Log等级总开关
    if not os.path.exists('./logfiles'): # 创建log目录
        os.mkdir('./logfiles')

    logfile = './logfiles/force_control.log' # 创建一个handler，用于写入日志文件
    fh = RotatingFileHandler(logfile, mode='a', maxBytes=1024*1024*50, backupCount=30) # 以append模式打开日志文件
    fh.setLevel(logging.INFO) # 输出到file的log等级的开关

    ch = logging.StreamHandler() # 再创建一个handler，用于输出到控制台
    ch.setLevel(logging.INFO) # 输出到console的log等级的开关

    formatter = logging.Formatter("%(asctime)s [%(thread)u] %(levelname)s: %(message)s") # 定义handler的输出格式
    fh.setFormatter(formatter) # 为文件输出设定格式
    ch.setFormatter(formatter) # 控制台输出设定格式

    logger.addHandler(fh) # 设置文件输出到logger
    logger.addHandler(ch) # 设置控制台输出到logger

def initialization():
    print("Socket library initialized successfully!")

# 定义报文头和尾
PACK_BEGIN = "<PACK_BEGIN"
PACK_END = "PACK_END>"
force_data = np.zeros(6) # fx fy fz Mx My Mz 原始力传感器数据
calib_force_data = np.zeros(6) # 标定后的力数据
force_zero_point = np.zeros(6) # 力传感器数据清零
# 线程池
executor = concurrent.futures.ThreadPoolExecutor(max_workers=5)

def process_data(packet):
    """
    解析和处理接收到的单个数据包
    """
    global force_data
    try:
        # 从数据包中解析 JSON 数据
        packet_content = packet[len(PACK_BEGIN): -len(PACK_END)]
        length_str = packet_content[:8].strip()
        length = int(length_str)
        json_data = packet_content[8:8 + length]
        json_obj = json.loads(json_data)
        
        # 更新全局力传感器数据
        force_data = json_obj["force_data"]
        logger.info(f"解析成功: {force_data}")
    
    except (ValueError, json.JSONDecodeError) as e:
        logger.error(f"数据解析失败: {e}")

def read_force_data(sock):
    """
    从 socket 读取数据
    """
    buffer = b""
    global read_force_data_running

    while read_force_data_running:
        try:
            data = sock.recv(4096)  # 接收数据
            if not data:
                break
            
            buffer += data
            # 检查数据包是否完整
            begin_pos = buffer.find(PACK_BEGIN.encode())
            end_pos = buffer.find(PACK_END.encode())
            
            while begin_pos >= 0 and end_pos >= 0 and end_pos > begin_pos + len(PACK_BEGIN):
                # 提取完整数据包
                packet = buffer[begin_pos:end_pos + len(PACK_END)]
                # 提交数据包给线程池进行解析和处理
                executor.submit(process_data, packet.decode())
                
                # 移动缓冲区指针
                buffer = buffer[end_pos + len(PACK_END):]
                begin_pos = buffer.find(PACK_BEGIN.encode())
                end_pos = buffer.find(PACK_END.encode())
        
        except Exception as e:
            logger.error(f"读取数据时发生错误: {e}")
            break

    # 断开力传感器连接
    sock.close()
    logger.info("程序结束, 断开力传感器socket连接")
    

def calib_force_data_func(robot):
    '''
    校准力传感器数据, 输出负载辨识和标定后的纯外力数据
    '''

    global calib_force_data,read_force_data_running, force_data, force_zero_point

    # 读取 calibration_result.JSON 文件得到标定结果
    file_name = './calibrate_forceSensor/calibration_result.json'
    with open(file_name, 'r') as json_file:
        json_data = json.load(json_file)
        gravity_bias = np.array(json_data['gravity_bias'])
        mass = np.array(json_data['mass'][0])
        force_zero = np.array(json_data['force_zero'])

    # 初始化一个队列用于存储前5次的力传感器数据
    length_force_data_queue = 20
    force_data_queue = deque(maxlen=length_force_data_queue)

    # 定义滤波器参数
    cutoff = 3  # 截止频率 (Hz)
    order = 2    # 滤波器阶数
    fs = 200  # 根据实际情况设置采样频率 (Hz)
    b, a = butter(order, cutoff / (0.5 * fs), btype='low', analog=False) # 设计低通Butterworth滤波器

    alpha = 0.1 # 滑动窗口加权滤波
    smoothed_force_data = np.zeros(6) # 平滑后的力传感器数据
    
    while read_force_data_running:
        
        if force_data is not None:
            
            raw_force_data = np.array(force_data).copy()

            # 将当前的力传感器数据添加到队列中
            force_data_queue.append(raw_force_data)
            # 如果队列中的数据少于5次，则继续等待
            if len(force_data_queue) < length_force_data_queue:
                continue
            # 将 force_data_queue 转换为 NumPy 数组
            force_data_array = np.array(force_data_queue)
            # 应用滤波器
            filtered_signal = filtfilt(b, a, force_data_array, axis=0)
            new_force_data = filtered_signal[-1]

            smoothed_force_data = alpha * new_force_data  + (1-alpha) * smoothed_force_data

            '''
            获取机械臂当前的末端姿态, 需要根据不同的机械臂型号和通信协议获取
            '''
            current_waypoint = robot.get_current_waypoint()
            # 当前末端位置 in m
            ee_pos = [np.round(i,6) for i in current_waypoint['pos']] 
            # 当前末端姿态
            ee_ori_rpy_rad = np.round(np.array(robot.quaternion_to_rpy(current_waypoint['ori'])),6) # in rad
            ee_ori_rpy_deg = np.round(ee_ori_rpy_rad/np.pi*180,2) # in degree

            '''
            计算外部力数据 = 原始力数据 - 力传感器零点 - 负载重力分量
            '''
            rotation_matrix = np.array(transform_utils.get_matrix_from_quaternion(current_waypoint['ori']))
            inv_rotation_matrix = np.linalg.inv(rotation_matrix) # 3*3
            temp_gravity_vector = np.array([0, 0, -1]).reshape(3, 1) # 3*1
            gravity_vector = np.dot(inv_rotation_matrix, temp_gravity_vector) # 3*1
            G_force[:3] = np.transpose(mass * gravity_vector) # 3*1 Gx Gy Gz
            G_force[3] = G_force[2] * gravity_bias[1] - G_force[1] * gravity_bias[2] # Mgx = Gz × y − Gy × z 
            G_force[4] = G_force[0] * gravity_bias[2] - G_force[2] * gravity_bias[0] # Mgy = Gx × z − Gz × x
            G_force[5] = G_force[1] * gravity_bias[0] - G_force[0] * gravity_bias[1] # Mgz = Gy × x − Gx × y

            # 标定后的力数据 = 原始力传感器数据 - 力传感器零点 - 负载重力分量 - 手动清零分量
            calib_force_data = smoothed_force_data - force_zero - G_force - force_zero_point # 6*1
            logger.info(f"calib_force_data: {calib_force_data}")
            time.sleep(0.005) # 5ms/次
        
        else :

            logger.info("force_data is None, 没有收到原始力传感器数据")
            time.sleep(0.005)
            # 断开机器人服务器链接

    robot.disconnect()
    Auboi5Robot.uninitialize() # 释放库资源
    logger.info("程序结束, 断开机器人socket连接")
        

def recv_full_message(sock, start_marker, end_marker):
    buffer = bytearray()
    while True:
        data = sock.recv(2000)
        if not data:
            return None
        buffer.extend(data)
        start = buffer.find(start_marker)
        if start == -1:
            continue  # 未找到起始标记，继续接收
        end = buffer.find(end_marker, start)
        if end == -1:
            continue  # 未找到结束标记，继续接收
        # 提取完整数据包并保留未处理数据
        packet = buffer[start:end+len(end_marker)]
        buffer = buffer[end+len(end_marker):]
        return packet

def rt_push_arm_state(sock_rt_push):

    # 定义发送缓冲区和接受缓冲区
    send_buf = bytearray(2000)
    recv_buf = bytearray(1550)

    joint_get1 = b"jointPos"
    joint_get2 = b"jointTagCurrentI"

    essid = bytearray(150)

    while rt_push_running:
        try:
            recv_buf = recv_full_message(sock_rt_push, b'<PACK_BEGIN', b'PACK_END>')
            if len(recv_buf) <= 0:
                print("接受失败！")
                break
            else:
                strstr_1 = recv_buf.find(joint_get1)
                strstr_2 = recv_buf.find(joint_get2)

                if strstr_1 != -1 and strstr_2 != -1:
                    essid = recv_buf[strstr_1 + 11:strstr_2 - 3]
                    essid_str = essid.decode("utf-8", errors='ignore')  # 忽略无效字节
                    try:
                        rt_joint = np.fromstring(essid_str, sep=",")
                        logger.info(rt_joint)
                    except ValueError as ve:
                        print(f"Error converting to numpy array: {ve}")
        except socket.error as e:
            print(f"接受失败: {e}")
            break
    
    # 关闭实时读取机器人状态的socket连接
    sock_rt_push.close()
    logger.info("程序结束, 断开实时读取机器人状态的socket连接")

if __name__ == "__main__":

    logger_init()

    # ======================================================   
    logger.info("机械臂实时状态推送线程启动")
    rt_push_running = True # 实时状态推送线程运行标志
    rt_joint = np.zeros(6) # 实时关节角度
    server_address = ('192.168.26.103', 8891) # 定义服务端地址
    try:
        sock_rt_push = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock_rt_push.connect(server_address)
        logger.info("机械臂实时状态读取连接成功！")
    except socket.error as e:
        logger.info(f"服务器连接失败: {e}") 
    rt_push_thread = threading.Thread(target=rt_push_arm_state,args=(sock_rt_push,))
    rt_push_thread.start()
    time.sleep(1) # 等待线程启动
    # ======================================================   

    # ======================================================   
    server_address = ('192.168.26.103', 8896)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(server_address)
    logger.info("连接力传感器成功")
    logger.info("读取原始外部力数据线程启动")
    read_force_data_running = True # 读取数据线程运行标志
    G_force = np.zeros(6) # Gx Gy Gz Mgx Mgy Mgz 负载重力在力传感器坐标系下的分量
    read_force_data_thread = threading.Thread(target=read_force_data,args=(sock,))
    read_force_data_thread.start()
    time.sleep(1) # 等待线程启动

    # socket连接机器人
    Auboi5Robot.initialize() 
    robot = Auboi5Robot()
    handle = robot.create_context() # 创建上下文
    result = robot.connect('192.168.26.103', 8899)
    robot.set_collision_class(6) # 设置碰撞等级
    robot.init_profile() # 初始化全局配置文件 自动清理掉之前设置的用户坐标系，速度，加速度等属性
    robot.set_joint_maxacc((0.8, 0.8, 0.8, 0.8, 0.8, 0.8)) # 设置关节最大加速度 rad/s^2
    robot.set_joint_maxvelc((0.8, 0.8, 0.8, 0.8, 0.8, 0.8)) # 设置关节最大速度 rad/s
    line_maxacc = 0.6 # 设置末端运动最大线加速度 m/s^2
    robot.set_end_max_line_acc(line_maxacc)
    line_maxvelc = 0.6 # 设置末端运动最大线速度 m/s
    robot.set_end_max_line_velc(line_maxvelc)
    trans_tcp2flange = SE3.Trans(0, 0, 0.211) # flange to tcp

    logger.info("读取校准外部力数据线程启动")
    calib_force_data_thread = threading.Thread(target=calib_force_data_func,args=(robot,))
    calib_force_data_thread.start()
    time.sleep(0.5) # 等待线程启动
    # ======================================================

    try:

        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:

        pass

    finally:
        # 结束读取力传感数据和机器人状态的线程
        read_force_data_running = False
        read_force_data_thread.join()
        calib_force_data_thread.join()

        rt_push_running = False
        rt_push_thread.join()
        logger.info("程序结束, 结束所有线程")



