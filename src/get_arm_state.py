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


# 创建一个logger
logger = logging.getLogger('calibration_forceSensor')

def logger_init():
    
    logger.setLevel(logging.INFO) # Log等级总开关
    if not os.path.exists('./logfiles'): # 创建log目录
        os.mkdir('./logfiles')
    logfile = './logfiles/get_arm_state.log' # 创建一个handler，用于写入日志文件
    fh = RotatingFileHandler(logfile, mode='a', maxBytes=1024*1024*50, backupCount=30) # 以append模式打开日志文件
    fh.setLevel(logging.INFO) # 输出到file的log等级的开关
    ch = logging.StreamHandler() # 再创建一个handler，用于输出到控制台
    ch.setLevel(logging.INFO) # 输出到console的log等级的开关
    formatter = logging.Formatter("%(asctime)s [%(thread)u] %(levelname)s: %(message)s") # 定义handler的输出格式
    fh.setFormatter(formatter) # 为文件输出设定格式
    ch.setFormatter(formatter) # 控制台输出设定格式
    logger.addHandler(fh) # 设置文件输出到logger
    logger.addHandler(ch) # 设置控制台输出到logger

def joint_status_callback(robot):
    print("joint_status_callback: ", robot.last_event)

if __name__ == '__main__':

    # 初始化logger
    logger_init()

    # 启动测试
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # 系统初始化
    Auboi5Robot.initialize()

    # 创建机械臂控制类
    robot = Auboi5Robot()

    # ==================pykin====================
    pykin = SingleArm(f_name="./description/aubo_i10.urdf")
    pykin.setup_link_name(base_name="base_link", eef_name="wrist3_Link")

    # 创建上下文
    handle = robot.create_context()

    try:

        # 链接服务器
        ip = 'localhost'
        ip = '192.168.26.103'

        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            
            # 设置碰撞等级
            robot.set_collision_class(7)

            # 获取机械臂状态
            st_time = time.time()
            current_waypoint = robot.get_current_waypoint()
            print('time get current joint: ',time.time()-st_time)

            # 当前关节角 
            real_joint_rad = np.round(np.array(current_waypoint['joint']),8) # in rad
            real_joint_deg = real_joint_rad/np.pi*180                        # in degree
            print(f"直接获取-关节joint in radian:{[rq for rq in real_joint_rad]}")
            print(f"直接获取-关节joint in degree:{[rq for rq in real_joint_deg]}")

            # 当前末端位置 in m
            ee_pos = [np.round(i,8) for i in current_waypoint['pos']]
            print(f"直接获取-末端position:{[rq for rq in ee_pos]} m")

            # 当前末端姿态
            ee_ori_rpy_rad = np.round(np.array(robot.quaternion_to_rpy(current_waypoint['ori'])),8) # in rad
            ee_ori_rpy_deg = np.round(ee_ori_rpy_rad/np.pi*180,8) # in degree
            print(f"直接获取-末端oritation:{[rq for rq in ee_ori_rpy_rad]} in radian")
            print(f"直接获取-末端oritation:{[rq for rq in ee_ori_rpy_deg]} in degree")

            print("\n==================自带正运动学库计算当前末端位置==================\n")
            fk_result = robot.forward_kin(joint_radian=list(real_joint_rad))

            # 当前末端位置 in m
            fk_ee_pos = [np.round(i,8) for i in fk_result['pos']]
            print(f"自带正运动学-末端位置:{[rq for rq in fk_ee_pos]} m")

            # 当前末端姿态
            fk_ee_ori_rpy_rad = np.round(np.array(robot.quaternion_to_rpy(fk_result['ori'])),8) # in rad
            fk_ee_ori_rpy_deg = np.round(ee_ori_rpy_rad/np.pi*180,8) # in degree
            print(f"自带正运动学-末端姿态:{[rq for rq in fk_ee_ori_rpy_rad]} in radian")
            print(f"自带正运动学-末端姿态:{[rq for rq in fk_ee_ori_rpy_deg]} in degree")

            print("\n==================pykin运动学库计算当前末端位置==================\n")
            pykin_result = pykin.forward_kin(real_joint_rad)

            # 当前末端位置 in m       
            py_ee_pos = np.round(np.array(pykin_result[pykin.eef_name].pos),8)
            print(f"pykin正运动学-末端位置:{[rq for rq in py_ee_pos]} m")

            # 当前末端姿态
            py_ee_ori_rpy_rad = np.round(np.array(transform_utils.get_rpy_from_quaternion(pykin_result[pykin.eef_name].rot)),8) # in rad
            py_ee_ori_rpy_deg = np.round(py_ee_ori_rpy_rad/np.pi*180,8) # in degree
            print(f"正运动学-末端姿态:{[rq for rq in py_ee_ori_rpy_rad]} in radian")
            print(f"正运动学-末端姿态:{[rq for rq in py_ee_ori_rpy_deg]} in degree")    


    except RobotError as e:
        logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))

    finally:

        # 断开机械臂链接
        if robot.connected:
            robot.disconnect()

        # 释放库资源
        Auboi5Robot.uninitialize()

        logger.info("{0} get_arm_state completed.".format(Auboi5Robot.get_local_time()))