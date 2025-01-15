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
import concurrent.futures
from spatialmath import SE3


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


def robot_inverse_kinematic(target_pos,target_ori_rpy_rad):
    '''
    逆运动学求解关节角驱动机械臂运动
    :param target_pos: 末端期望位置 / m
    :param target_ori_rpy_rad 末端期望角度rqy / degree
    :return: 逆运动学求解的关节角度 / rad
    '''
    global robot
    current_waypoint = robot.get_current_waypoint()
    current_joint_rad = np.array(current_waypoint['joint']) # in rad
    target_rot = transform_utils.get_matrix_from_rpy(target_ori_rpy_rad)
    target_pose = np.eye(4);target_pose[:3, :3] = target_rot; target_pose[:3, 3] = target_pos
    r_qd = pykin.inverse_kin(current_joint_rad, target_pose, method="LM", max_iter=20)

    # 轴动到初始位置
    desired_joint = tuple(r_qd)
    return desired_joint


if __name__ == '__main__':
 
    logger_init()
    
    # =====================机器人连接=========================
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
    trans_flange2tcp = SE3.Trans(0, 0, 0.211) # flange to tcp
    # ======================================================

    # =========================pykin=========================
    pykin = SingleArm(f_name="./description/aubo_i10.urdf")
    pykin.setup_link_name(base_name="base_link", eef_name="Tool_Link")
    # ======================================================


    # ==========================主程序=========================
    try:

        # 获取机械臂状态
        current_waypoint = robot.get_current_waypoint() 
        real_joint_rad = np.array(current_waypoint['joint']) # in rad
        print(f"直接获取-关节joint in radian:{[rq for rq in real_joint_rad]}")

        # 当前末端位置 in m
        ee_pos = np.array(current_waypoint['pos'])
        print(f"直接获取-末端position:{[rq for rq in ee_pos]} m")

        # 当前末端姿态
        ee_ori_rpy_rad = np.array(robot.quaternion_to_rpy(current_waypoint['ori'])) # in rad
        print(f"直接获取-末端oritation:{[rq for rq in ee_ori_rpy_rad]} in radian")

        flange_pose = SE3.Trans(ee_pos[0], ee_pos[1], ee_pos[2]) * SE3.RPY(ee_ori_rpy_rad[0], ee_ori_rpy_rad[1], ee_ori_rpy_rad[2])
        tcp_pose = flange_pose * trans_flange2tcp
        tcp_pos = tcp_pose.t; tcp_ori = tcp_pose.rpy()
        tcp_mat = tcp_pose.R

        print('tcp_pos: ', tcp_pos)
        print('tcp_ori: ', tcp_ori)
        print('tcp_mat: ', tcp_mat)

        # trans_inTcpFrame = SE3.Trans(0.0,0.0,0.0) * SE3.RPY(0,0,10/180*np.pi)
        # target_pose = tcp_pose * trans_inTcpFrame
        # p_start_pos = target_pose.t
        # p_start_ori = target_pose.rpy()

        p_start_pos = tcp_pos.copy() + np.array([0.0, 0.002, 0.0])
        p_start_ori = np.array([-90/180*np.pi, -76/180*np.pi, -180/180*np.pi])

        # p_start_pos = tcp_pos.copy() - np.array([0.0, 0.0, 0.02])
        # p_start_ori = tcp_ori.copy()

        desired_joint = robot_inverse_kinematic(p_start_pos,p_start_ori) # 末端期望位置 / m， 角度rqy / rad
        print(f"逆运动学求解-关节joint in radian:{[rq for rq in desired_joint]}")
        robot.move_joint(desired_joint)
        
    except KeyboardInterrupt: # CTRL+C退出程序

        logger.info("程序被中断")

    except Exception as e: # 其他异常

        logger.info("程序异常退出: ", e)

    finally:


        logger.info("程序结束, 关闭机器人连接")

        # 断开机器人服务器链接
        robot.disconnect()
        Auboi5Robot.uninitialize() # 释放库资源