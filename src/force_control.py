import os
import sys
import logging
from logging.handlers import RotatingFileHandler
import numpy as np
import matplotlib.pyplot as plt
import time
import socket
import threading
import json
from lib.robotcontrol import Auboi5Robot, RobotError, RobotErrorType, RobotEventType, RobotEvent, RobotMoveTrackType, RobotCoordType, RobotIOType, RobotUserIoName
from pykin.robots.single_arm import SingleArm
from pykin.utils import transform_utils as transform_utils
from collections import deque
from mpl_toolkits.mplot3d import Axes3D
from admittance_controller import AdmittanceController
from scipy.signal import butter, filtfilt
from scipy.interpolate import CubicSpline
import rospy
from std_msgs.msg import Float64MultiArray
import queue


# 创建一个logger
logger = logging.getLogger('calibration_forceSensor')

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



# 定义报文头和尾
PACK_BEGIN = "<PACK_BEGIN"
PACK_END = "PACK_END>"
calib_force_data = np.zeros(6) # 标定后的力数据
force_zero_point = np.zeros(6) # 力传感器数据清零

def read_force_data(sock, robot):
    '''
    读取原始力传感器数据，并输出负载辨识和标定后的纯外力数据 5ms/次
    '''
    # 读取 calibration_result.JSON 文件得到标定结果
    file_name = './calibrate_forceSensor/calibration_result.json'
    with open(file_name, 'r') as json_file:
        json_data = json.load(json_file)
        gravity_bias = np.array(json_data['gravity_bias'])
        mass = np.array(json_data['mass'][0])
        force_zero = np.array(json_data['force_zero'])

    buffer = b""
    start_time = time.time()

    global read_data_running, force_zero_point, calib_force_data

    # 初始化一个队列用于存储前5次的力传感器数据
    length_force_data_queue = 10
    force_data_queue = deque(maxlen=length_force_data_queue)

    # 定义滤波器参数
    cutoff = 4  # 截止频率 (Hz)
    order = 2    # 滤波器阶数
    fs = 200  # 根据实际情况设置采样频率 (Hz)
    b, a = butter(order, cutoff / (0.5 * fs), btype='low', analog=False) # 设计低通Butterworth滤波器

    alpha = 0.8 # 滑动窗口加权滤波
    smoothed_force_data = np.zeros(6) # 平滑后的力传感器数据
    
    while read_data_running:

        data = sock.recv(4096)
        if not data:
            return None

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
                force_data = json_obj["force_data"] # fx fy fz Mx My Mz 力传感器数据
                buffer = buffer[end_pos + len(PACK_END.encode()):]
                
            except json.JSONDecodeError as e:

                logger.info("JSON解析错误:", e)
                buffer = buffer[end_pos + len(PACK_END.encode()):]
                force_data = None
        
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
            ee_pos = [np.round(i,4) for i in current_waypoint['pos']] 
            # 当前末端姿态
            ee_ori_rpy_rad = np.round(np.array(robot.quaternion_to_rpy(current_waypoint['ori'])),4) # in rad
            ee_ori_rpy_deg = np.round(ee_ori_rpy_rad/np.pi*180,2) # in degree
            pose_data = np.concatenate((ee_pos, ee_ori_rpy_deg), axis=0) # 6*1

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
            
            # 保存数据
            force_data_list.append(calib_force_data) # 末端受到的外部力
            pose_data_list.append(pose_data) # 末端位置和姿态
            time_list.append(time.time() - start_time) # 时间

            time.sleep(0.004) # 5ms/次

def force_data_zero():
    '''
    力传感器数据清零
    '''
    global force_zero_point, calib_force_data
    force_zero_point = calib_force_data.copy()

def pose_add(pose1, pose2):
    '''
    位姿相加函数，计算两个位姿的和，位置直接相加，姿态用四元数相加: q1 + q2 = q2 * q1
    param pose1: 位姿1 [x1,y1,z1,rx1,ry1,rz1]
    param pose2: 位姿2 [x2,y2,z2,rx2,ry2,rz2]
    return: 位姿 [x1+x2,y1+y2,z1+z2,rx,ry,rz] = pose1 + pose2
    '''
    # 位置直接相加
    pose1 = np.array(pose1); pose2 = np.array(pose2)
    pose = pose1.copy()
    pose[:3] = pose1[:3] + pose2[:3]

    # 姿态用四元数相加
    quater1 = transform_utils.get_quaternion_from_rpy(pose1[3:])
    quater2 = transform_utils.get_quaternion_from_rpy(pose2[3:])
    quater3 = transform_utils.quaternion_multiply(quater2, quater1) # 顺序不能反，先施加quater1再施加quater2,所以quater1在前面
    pose[3:] = transform_utils.get_rpy_from_quaternion(quater3)

    return pose


def pose_sub(pos1, pos2):
    '''
    位姿相减函数，计算两个位姿的差，位置直接相减，姿态用四元数相减: q1 - q2 = q2' * q1
    param pos1: 位姿1 [x1,y1,z1,rx1,ry1,rz1]
    param pos2: 位姿2 [x2,y2,z2,rx2,ry2,rz2]
    return: 位姿 [x1-x2,y1-y2,z1-z2,rx,ry,rz] = pos1 - pos2 
    '''
    # 位置直接相减
    pose1 = np.array(pos1); pose2 = np.array(pos2)
    pose = pose1.copy()
    pose[:3] = pose1[:3] - pose2[:3]

    # 姿态用四元数相减
    quater1 = transform_utils.get_quaternion_from_rpy(pose1[3:])
    temp_quater2 = transform_utils.get_quaternion_from_rpy(pose2[3:])
    quater2 = temp_quater2.copy(); quater2[1:] = -quater2[1:] # 逆四元数
    quater3 = transform_utils.quaternion_multiply(quater1, quater2) # 顺序不能反，从quater2旋转到quater1,所以quater2在后面
    pose[3:] = transform_utils.get_rpy_from_quaternion(quater3)

    return pose  

def robot_inverse_kinematic(target_pos,target_ori_rpy_rad):
    '''
    逆运动学求解关节角驱动机械臂运动
    :param target_pos: 末端期望位置 / m
    :param target_ori_rpy_rad 末端期望角度rqy / degree
    :return: 逆运动学求解的关节角度 / rad
    '''
    global robot
    current_waypoint = robot.get_current_waypoint()
    current_joint_rad = np.round(np.array(current_waypoint['joint']),4) # in rad
    target_rot = transform_utils.get_matrix_from_rpy(target_ori_rpy_rad)
    target_pose = np.eye(4);target_pose[:3, :3] = target_rot; target_pose[:3, 3] = target_pos
    r_qd = pykin.inverse_kin(current_joint_rad, target_pose, method="LM", max_iter=10)

    # 轴动到初始位置
    desired_joint = tuple(r_qd)
    return desired_joint


# Function to compute and generate the 5th-order polynomial trajectory
def generate_trajectory(q_start, v_start, a_start, q_end, v_end, a_end, move_period, lookahead_time):
    """
    Generate 5th-order polynomial trajectory for all joints.
    :param q_start: Initial joint positions
    :param v_start: Initial joint velocities
    :param a_start: Initial joint accelerations
    :param q_end: Target joint positions
    :param v_end: Target joint velocities
    :param a_end: Target joint accelerations
    :param move_period: Time interval for trajectory sampling
    :param lookahead_time: Total time for trajectory planning
    :return: Position, velocity, and acceleration trajectories for all joints
    """
    num_joints = len(q_start)
    t = np.arange(0, lookahead_time, move_period)  # Time steps

    # Initialize trajectory arrays
    positions = np.zeros((len(t), num_joints))
    velocities = np.zeros((len(t), num_joints))
    accelerations = np.zeros((len(t), num_joints))

    for j in range(num_joints):
        # Compute coefficients for 5th-order polynomial
        A = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [1, lookahead_time, lookahead_time**2, lookahead_time**3, lookahead_time**4, lookahead_time**5],
            [0, 1, 2*lookahead_time, 3*lookahead_time**2, 4*lookahead_time**3, 5*lookahead_time**4],
            [0, 0, 2, 6*lookahead_time, 12*lookahead_time**2, 20*lookahead_time**3]
        ])
        B = np.array([q_start[j], v_start[j], a_start[j], q_end[j], v_end[j], a_end[j]])
        coeffs = np.linalg.solve(A, B)

        # Generate position, velocity, and acceleration
        positions[:, j] = coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5
        velocities[:, j] = coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t**2 + 4*coeffs[4]*t**3 + 5*coeffs[5]*t**4
        accelerations[:, j] = 2*coeffs[2] + 6*coeffs[3]*t + 12*coeffs[4]*t**2 + 20*coeffs[5]*t**3

    return positions, velocities, accelerations

def admittance_controller():
    '''
    阻抗控制器, 用于机械臂外环控制, 控制周期25hz
    '''

    global cur_joint,cur_joint_vel,cur_joint_acc,flag_enter_tco2canbus

    # 设置admittance control阻抗控制器参数
    mass = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # 定义惯性参数
    stiffness = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]  # 定义刚度参数
    damping_ratio = 0.8  # # 定义阻尼比（ξ）
    control_period = 0.04 # 阻抗控制周期（单位：秒）要跟状态反馈时间保持一致，用于外环阻抗控制循环

    # 初始化导纳参数
    controller = AdmittanceController(mass, stiffness, damping_ratio, control_period)

    # 期望末端位置和速度
    des_eef_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    des_eef_vel = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # robot.move_joint(tuple([1.5756, -0.0999, -1.7131, 1.5337, 1.6265, 3.0839]))

    # 获取当前末端位姿
    current_waypoint = robot.get_current_waypoint()
    current_joint_rad = np.round(np.array(current_waypoint['joint']),4) # in rad
    ee_pos = np.array([np.round(i,4) for i in current_waypoint['pos']]) # 当前末端位置 in m
    ee_ori_quater = np.round(np.array(current_waypoint['ori']),4) # 当前末端姿态 in quaternion
    ee_ori_rpy_rad = np.round(np.array(robot.quaternion_to_rpy(ee_ori_quater)),4) # # 当前末端姿态 in rad

    # 模拟圆弧轨迹规划
    radius = 0.06  # 圆的半径
    p_start = ee_pos.copy() # 起点位置
    p_center = ee_pos.copy() # 圆心位置为起点
    # p_start[0] = p_start[0] + radius # 移动到最高点

    desired_joint = robot_inverse_kinematic(p_start,ee_ori_rpy_rad) # 末端期望位置 / m， 角度rqy / rad
    print('desired_joint: ', [r_qd/np.pi*180 for r_qd in desired_joint])
    robot.move_joint(desired_joint)
    time.sleep(1)
    
    # 进入 TCP 转 CAN 透传模式
    result = robot.enter_tcp2canbus_mode()
    if result != RobotErrorType.RobotError_SUCC:
        logger.info("TCP 转 CAN 透传模式失败, 错误码：{}".format(result))
    else:
        logger.info("TCP 转 CAN 透传模式成功")
        flag_enter_tco2canbus = True


    # 先进行路径规划, 规划每一个目标点的位姿
    num_points = 1000  # 圆轨迹上的点数
    angles = np.linspace(0, 2 * np.pi, num_points) # 计算圆轨迹上的点

    for angle in angles:

        # x = p_center[0] + radius * np.cos(angle)
        # y = p_center[1]
        # z = p_center[2] + radius * np.sin(angle)
        x = p_start[0]
        y = p_start[1]
        z = p_start[2]
        rx = ee_ori_rpy_rad[0]
        ry = ee_ori_rpy_rad[1]
        rz = ee_ori_rpy_rad[2]
        p_end = np.array([x, y, z, rx, ry, rz])
        p_end_list.append(p_end)

    p_end_array = np.array(p_end_list)
    num_points = p_end_array.shape[0] # 轨迹上点的数量

    '''
    开始规划期望轨迹并跟踪
    '''
    force_data_zero()

    logger.info("进入外环导纳控制周期，控制周期{}ms".format(control_period*1000))
    for i in range(num_points):

        st_time = time.time()
        
        # 获取当前关节状态和末端状态
        current_waypoint = robot.get_current_waypoint()
        cur_joint = np.array(current_waypoint['joint']) # in rad 
        ee_pos = np.array(current_waypoint['pos']) # 当前末端位置 in m
        ee_ori_quater = np.array(current_waypoint['ori']) # 当前末端姿态 in quaternion
        ee_ori_rpy_rad = np.array(robot.quaternion_to_rpy(ee_ori_quater)) # # 当前末端姿态 in rad

        # # 等待并获取最新的 joint_angles 话题消息
        # joint_msg = rospy.wait_for_message('joint_angles', Float64MultiArray)
        # cur_joint = np.array(joint_msg.data) # in rad
        all_positions_real.append(cur_joint) # in rad

        # 获取当前末端位置和姿态
        controller.eef_pose = np.concatenate((ee_pos, ee_ori_rpy_rad), axis=0) # 6*1

        # 根据规划的路径获取旧的末端期望位姿
        des_eef_pose = p_end_array[i, :]

        # 将传感器坐标系下的力转换到基坐标系下，注意此时受力坐标系为传感器坐标系，不是末端坐标系
        py_ee_transmat = np.array(transform_utils.get_matrix_from_quaternion(ee_ori_quater)) # 3*3 末端姿态矩阵
        contact_force = py_ee_transmat @ calib_force_data[:3] # 末端受到的外部力转换到基坐标系下
        contact_troque = py_ee_transmat @ calib_force_data[3:] # 末端受到的外部力矩转换到基坐标系下
        contact_force_data = np.concatenate((contact_force, contact_troque), axis=0) # 6*1

        # 通过导纳控制器计算新的末端期望位姿
        updated_eef_pose = controller.update(des_eef_pose, des_eef_vel, contact_force_data)

        # 设置期望位置、速度和加速度
        q_target = robot_inverse_kinematic(updated_eef_pose[:3],updated_eef_pose[3:])
        v_target = np.zeros(num_joints)  # Assume target velocities are zero
        a_target = np.zeros(num_joints)  # Assume target accelerations are zero

        # 5次多项式轨迹规划生成平滑轨迹
        positions, velocities, accelerations = generate_trajectory(
            cur_joint, cur_joint_vel, cur_joint_acc,
            q_target, v_target, a_target,
            move_period, lookahead_time
        )

        # 将规划好的轨迹放进共享队列
        trajectory_queue.put((positions,velocities,accelerations))
        
        # 时间补偿, 保证控制周期为control_period
        elapsed_time = time.time()-st_time # 计算消耗时间=轨迹规划时间+逆运动学求解时间
        if elapsed_time > control_period:
            pass
        else:
            time.sleep(control_period - elapsed_time)

    # 退出 TCP 转 CAN 透传模式
    flag_enter_tco2canbus = False
    robot.leave_tcp2canbus_mode()
    logger.info("结束外环控制周期，退出 TCP 转 CAN 透传模式")


def servo_j():
    '''
    机械臂内环位置控制, 控制周期10ms
    '''

    global cur_joint,cur_joint_vel,cur_joint_acc,flag_enter_tco2canbus

    traj_data = None
    traj_index = 0
    traj_len = 0

    logger.info("进入内环位置控制周期，控制周期{}ms".format(move_period*1000))

    while servo_j_running:

        st_time = time.time()

        # 获取轨迹的逻辑：当前轨迹为空 or 当前轨迹已经执行完毕 or 轨迹队列中有新的轨迹
        if traj_data is None or traj_index >= traj_len or not trajectory_queue.empty():

            # 从共享队列中获取轨迹数据
            while not trajectory_queue.empty():
                traj_data = trajectory_queue.get()

            # 获取了新的轨迹数据
            if traj_data: 
                positions, velocities, accelerations = traj_data
                traj_index = 0
                traj_len = positions.shape[0]
        
        # 如果没有轨迹，就再等一下
        if traj_data is None:
            time.sleep(move_period)
            continue

        # 直接can协议透传目标位置
        if flag_enter_tco2canbus and traj_index < traj_len:
                    
            robot.set_waypoint_to_canbus(tuple(positions[traj_index,:])) # 透传目标位置给机械臂
            all_positions_desired.append(positions[traj_index,:]) # 保存期望关节角度
            cur_joint = positions[traj_index,:] # 更新当前关节角度
            cur_joint_vel = velocities[traj_index,:] # 更新当前关节角速度
            cur_joint_acc = accelerations[traj_index,:] # 更新当前关节角加速度
            traj_index += 1 # 更新轨迹索引

        else:
            pass # 轨迹已经播放结束，等待下一个周期

        # 时间补偿, 保证内环控制周期为move_period
        elapsed_time = time.time()-st_time
        if elapsed_time > move_period:
            pass
        else:
            time.sleep(move_period - elapsed_time)
    
    logger.info("内环位置控制周期结束")
    
if __name__ == '__main__':
 
    logger_init()

    # ==================socket连接力传感器====================
    server_address = ('192.168.26.103', 8896)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(server_address)
    logger.info("连接力传感器成功")
    
    # =====================机器人连接=========================
    Auboi5Robot.initialize() 
    robot = Auboi5Robot()
    handle = robot.create_context() # 创建上下文
    result = robot.connect('192.168.26.103', 8899)
    if result != RobotErrorType.RobotError_SUCC:
        logger.info("连接机器人失败")
    else:
        logger.info("连接机器人成功")
    robot.set_collision_class(6) # 设置碰撞等级
    robot.init_profile() # 初始化全局配置文件 自动清理掉之前设置的用户坐标系，速度，加速度等属性
    robot.set_joint_maxacc((2.5, 2.5, 2.5, 2.5, 2.5, 2.5)) # 设置关节最大加速度 rad/s^2
    robot.set_joint_maxvelc((2, 2, 2, 2, 2, 2)) # 设置关节最大速度 rad/s
    line_maxacc = 0.6 # 设置末端运动最大线加速度 m/s^2
    robot.set_end_max_line_acc(line_maxacc)
    line_maxvelc = 0.6 # 设置末端运动最大线速度 m/s
    robot.set_end_max_line_velc(line_maxvelc)

    # =========================pykin=========================
    pykin = SingleArm(f_name="./description/aubo_i10.urdf")
    pykin.setup_link_name(base_name="base_link", eef_name="wrist3_Link")

    
    # =====================保存读取数据初始化===================
    start_time = time.time() # 记录程序开始时间
    force_data_list = [] # 记录每次读取的力数据
    pose_data_list = [] # 记录每次读取的末端位姿
    joint_data_list = [] # 记录每次读取的关节角度
    p_end_list = [] # 记录每次规划的末端位置
    time_list = [] # 记录每次读取的时间
    read_data_running = True # 读取数据线程运行标志
    G_force = np.zeros(6) # Gx Gy Gz Mgx Mgy Mgz 负载重力在力传感器坐标系下的分量


    # ========================读取外部力数据====================
    # 单开一个线程读取力传感器数据
    read_data_thread = threading.Thread(target=read_force_data, args=(sock,robot))
    read_data_thread.start()
    logger.info("等待读取外部力数据线程启动")

    # # 初始化ROS节点
    # rospy.init_node('subscribe_joint_state', anonymous=True)

    # ===================设置 servo_j 内环位置控制参数===============
    trajectory_queue = queue.Queue() # 初始化轨迹队列
    move_period = 0.01  # 内环控制周期 10ms
    lookahead_time = 0.2  # Lookahead time (s) 前瞻时间，用0.2s规划轨迹
    num_joints = 6  # Number of joints
    cur_joint = np.zeros(num_joints) # 当前关节角 [0, 0, 0, 0, 0, 0]
    cur_joint_vel = np.zeros(num_joints) # 当前关节角速度 [0, 0, 0, 0, 0, 0]
    cur_joint_acc = np.zeros(num_joints) # 当前关节角加速度 [0, 0, 0, 0, 0, 0]
    servo_j_running = True # servo_j 线程运行标志
    flag_enter_tco2canbus = False # 是否进入 TCP 转 CAN 透传模式
    # 记录整段轨迹的期望关节角度和实际关节角度    
    all_positions_desired = []
    all_positions_real = []

    # ==========================主程序=========================
    try:

        servo_j_thread = threading.Thread(target=servo_j)
        servo_j_thread.start()
        admittance_controller() # 导纳控制器
        
    except KeyboardInterrupt: # CTRL+C退出程序

        logger.info("程序被中断")

    except Exception as e: # 其他异常

        logger.info("程序异常退出: ", e)

    finally:
        
        # 将 all_positions_desired 中的 NumPy 数组转换为 Python 列表
        all_positions_converted_desired = [pos.tolist() for pos in all_positions_desired]
        all_positions_real_converted = [pos.tolist() for pos in all_positions_real]

        with open('trajectory_data.json', 'w') as f:
            json.dump(all_positions_converted_desired, f)
        
        with open('trajectory_data_real.json', 'w') as f:
            json.dump(all_positions_real_converted, f)

        # 结束内环servoj线程
        servo_j_running = False
        servo_j_thread.join()

        # 结束读取力传感数据和机器人状态的线程
        read_data_running = False
        read_data_thread.join()

        logger.info("程序结束, 关闭socket连接和机器人连接")

        # 断开机器人服务器链接
        robot.disconnect()
        Auboi5Robot.uninitialize() # 释放库资源

        # 断开力传感器连接
        sock.close()

        logger.info("程序结束, 绘制接触力/力矩和末端位置/姿态")

        # 将 force_data_list 中的 NumPy 数组转换为 Python 列表
        force_data_list_converted = [data.tolist() for data in force_data_list]

        # 将力数据保存为 JSON 文件
        with open('force_data.json', 'w') as f:
            json.dump(force_data_list_converted, f)

        # 画图
        max_time = np.round(max(time_list)+1).astype(int)
        fig, (ax1, ax2) = plt.subplots(2, 1)
        
        lines1 = [ax1.plot([], [], label=label, color=color)[0] for label, color in zip(['fx', 'fy', 'fz'], ['r', 'g', 'b'])]
        lines2 = [ax2.plot([], [], label=label, color=color)[0] for label, color in zip(['mx', 'my', 'mz'], ['r', 'g', 'b'])]

        # 显示数据
        ax1.set_ylabel('F/N')
        ax1.legend()
        
        ax2.set_xlabel('Time/s')
        ax2.set_ylabel('M/Nm')
        ax2.legend()

        if len(time_list) == len(force_data_list) and force_data_list:
            data = np.array(force_data_list)  # 转换为numpy数组
            times = np.array(time_list) 
            
            if len(times) == data.shape[0]:
                for i, line in enumerate(lines1):
                    line.set_xdata(times)
                    line.set_ydata(data[:, i])
                
                for i, line in enumerate(lines2):
                    line.set_xdata(times)
                    line.set_ydata(data[:, i + 3])
                
                ax1.set_xlim(0, max_time)
                ax1.set_ylim(np.min(data[:, :3]) - 1, np.max(data[:, :3]) + 1)
                
                ax2.set_xlim(0, max_time)
                ax2.set_ylim(np.min(data[:, 3:]) - 1, np.max(data[:, 3:]) + 1)

        plt.tight_layout()
                
        # 画末端位置和姿态图
        fig2, (ax3, ax4) = plt.subplots(2, 1)
        
        lines3 = [ax3.plot([], [], label=label, color=color)[0] for label, color in zip(['x', 'y', 'z'], ['r', 'g', 'b'])]
        lines4 = [ax4.plot([], [], label=label, color=color)[0] for label, color in zip(['rx', 'ry', 'rz'], ['r', 'g', 'b'])]

        ax3.set_ylabel('Pos/m')
        ax3.legend()
        
        ax4.set_xlabel('Time/s')
        ax4.set_ylabel('Ori/degree')
        ax4.legend()

        if len(time_list) == len(pose_data_list) and pose_data_list:
            pose_data = np.array(pose_data_list)  # 转换为numpy数组
            times = np.array(time_list)         

            if len(times) == pose_data.shape[0]:
                for i, line in enumerate(lines3):
                    line.set_xdata(times)
                    line.set_ydata(pose_data[:, i])
                
                for i, line in enumerate(lines4):
                    line.set_xdata(times)
                    line.set_ydata(pose_data[:, i + 3])
                
                ax3.set_xlim(0, max_time)
                ax3.set_ylim(np.min(pose_data[:, :3]) - 0.5, np.max(pose_data[:, :3]) + 0.5)
                
                ax4.set_xlim(0, max_time)
                ax4.set_ylim(np.min(pose_data[:, 3:]) - 10, np.max(pose_data[:, 3:]) + 10)

        plt.tight_layout()

        # 创建一个三维图形
        fig3 = plt.figure()
        ax = fig3.add_subplot(111, projection='3d')
        p_end_array = np.array(p_end_list)

        # 绘制末端实际位置的三维轨迹（蓝色）
        ax.plot(pose_data[:, 0], pose_data[:, 1], pose_data[:, 2], color='b', label='End Effector Position')
        # 绘制末端期望位置的三维轨迹（红色）
        ax.plot(p_end_array[:, 0], p_end_array[:, 1], p_end_array[:, 2], color='r', label='Desired End Effector Position')
        
        # 设置三维图形的标签
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')
        ax.set_title('End Effector Position Trajectory')
        ax.legend()
        plt.show()

