import time
import logging
from logging.handlers import RotatingFileHandler
from multiprocessing import Process, Queue
import os
from math import pi
from lib.robotcontrol import Auboi5Robot, RobotError, RobotErrorType, RobotEventType, RobotEvent, RobotMoveTrackType, RobotCoordType, RobotIOType, RobotUserIoName
import numpy as np
from pykin.robots.single_arm import SingleArm
from pykin.utils import transform_utils as transform_utils

# 创建一个logger
#logger = logging.getLogger()

logger = logging.getLogger('test.robotcontrol')
current_path = os.path.dirname(os.path.abspath(__file__))


def logger_init():
    # Log等级总开关
    logger.setLevel(logging.INFO)

    # 创建log目录
    if not os.path.exists('./logfiles'):
        os.mkdir('./logfiles')

    # 创建一个handler，用于写入日志文件
    logfile = './logfiles/robot-ctl-python.log'

    # 以append模式打开日志文件
    # fh = logging.FileHandler(logfile, mode='a')
    fh = RotatingFileHandler(logfile, mode='a', maxBytes=1024*1024*50, backupCount=30)

    # 输出到file的log等级的开关
    fh.setLevel(logging.INFO)

    # 再创建一个handler，用于输出到控制台
    ch = logging.StreamHandler()

    # 输出到console的log等级的开关
    ch.setLevel(logging.INFO)

    # 定义handler的输出格式
    # formatter = logging.Formatter("%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s")
    formatter = logging.Formatter("%(asctime)s [%(thread)u] %(levelname)s: %(message)s")

    # 为文件输出设定格式
    fh.setFormatter(formatter)

    # 控制台输出设定格式
    ch.setFormatter(formatter)

    # 设置文件输出到logger
    logger.addHandler(fh)

    # 设置控制台输出到logger
    logger.addHandler(ch)

# 测试函数
def test(test_count=1):

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
    print(pykin.robot_name)

    # 创建上下文
    handle = robot.create_context()

    # 打印上下文
    logger.info("robot.rshd={0}".format(handle))

    try:

        # 链接服务器
        ip = 'localhost'
        ip = '192.168.26.103'

        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            # # 重新上电
            #robot.robot_shutdown()
            #
            # # 上电 松刹车
            # robot.robot_startup()
            #
            # # 设置碰撞等级
            robot.set_collision_class(7)

            # 设置工具端电源为１２ｖ
            # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)

            # 设置工具端ＩＯ_0为输出
            #robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

            # 获取工具端ＩＯ_0当前状态
            #tool_io_status = robot.get_tool_io_status(RobotToolIoName.tool_io_0)
            #logger.info("tool_io_0={0}".format(tool_io_status))

            # 设置工具端ＩＯ_0状态
            #robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)


            # 获取控制柜用户DO
            #io_config = robot.get_board_io_config(RobotIOType.User_DO)

            # 输出DO配置
            #logger.info(io_config)

            # 当前机械臂是否运行在联机模式
            #logger.info("robot online mode is {0}".format(robot.is_online_mode()))

            # 循环测试
            while test_count > 0:
                test_count -= 1

                #=========================== 直接获取机械臂当前位置信息
                current_waypoint = robot.get_current_waypoint()

                # 当前关节角 
                real_joint_rad = np.round(np.array(current_waypoint['joint']),4) # in rad
                real_joint_deg = real_joint_rad/np.pi*180                        # in degree
                print(f"直接获取-关节joint in radian:{real_joint_rad}")
                print(f"直接获取-关节joint in degree:{real_joint_deg}")

                # 当前末端位置 in m
                ee_pos = [np.round(i,4) for i in current_waypoint['pos']]
                print(f"直接获取-末端position:{ee_pos} m")

                # 当前末端姿态
                ee_ori_rpy_rad = np.round(np.array(robot.quaternion_to_rpy(current_waypoint['ori'])),4) # in rad
                ee_ori_rpy_deg = np.round(ee_ori_rpy_rad/np.pi*180,4) # in degree
                print(f"直接获取-末端oritation:{ee_ori_rpy_rad} in radian")
                print(f"直接获取-末端oritation:{ee_ori_rpy_deg} in degree")

                print("\n==================自带正运动学库计算当前末端位置==================\n")
                fk_result = robot.forward_kin(joint_radian=list(real_joint_rad))

                # 当前末端位置 in m
                fk_ee_pos = [np.round(i,4) for i in fk_result['pos']]
                print(f"自带正运动学-末端位置:{fk_ee_pos} m")

                # 当前末端姿态
                fk_ee_ori_rpy_rad = np.round(np.array(robot.quaternion_to_rpy(fk_result['ori'])),4) # in rad
                fk_ee_ori_rpy_deg = np.round(ee_ori_rpy_rad/np.pi*180,4) # in degree
                print(f"自带正运动学-末端姿态:{fk_ee_ori_rpy_rad} in radian")
                print(f"自带正运动学-末端姿态:{fk_ee_ori_rpy_deg} in degree")

                print("\n==================pykin运动学库计算当前末端位置==================\n")
                pykin_result = pykin.forward_kin(real_joint_rad)
                print(pykin_result)

                # 当前末端位置 in m       
                py_ee_pos = np.round(np.array(pykin_result[pykin.eef_name].pos),4)
                print(f"pykin正运动学-末端位置:{py_ee_pos} m")

                # 当前末端姿态
                py_ee_ori_rpy_rad = np.round(np.array(transform_utils.get_rpy_from_quaternion(pykin_result[pykin.eef_name].rot)),4) # in rad
                py_ee_ori_rpy_deg = np.round(py_ee_ori_rpy_rad/np.pi*180,4) # in degree
                print(f"正运动学-末端姿态:{py_ee_ori_rpy_rad} in radian")
                print(f"正运动学-末端姿态:{py_ee_ori_rpy_deg} in degree")    



                # 获取 IO 配置
                io_type = 5  # 控制柜-通用I/O-数字-输出 Digital Output
                board_io_config = robot.get_board_io_config(io_type)

                # 设置 IO 状态
                io_type = 5
                io_name = "U_DO_00"
                io_value = 1
                result = robot.set_board_io_status(io_type, io_name, io_value)
                time.sleep(1)
                if result != RobotErrorType.RobotError_SUCC:
                    logger.info("设置 IO 状态失败。错误码：{0}。".format(result))
                else:
                    logger.info("设置 IO 状态成功。类型：{0}, 名称：{1}, 状态：{2} ".format(io_type, io_name, io_value))

                # 获取 IO 状态
                io_type = 5
                io_name = "U_DO_01"
                board_io_status = robot.get_board_io_status(io_type, io_name)
                logger.info("获取 IO 状态：{0} ".format(board_io_status))

                # # 初始化全局配置文件 自动清理掉之前设置的用户坐标系，速度，加速度等属性
                # robot.init_profile()

                # # 设置关节最大加速度 rad/s^2
                # robot.set_joint_maxacc((1.0, 1.0, 1.0, 1.0, 1.0, 1.0))

                # # 设置关节最大速度 rad/s
                # robot.set_joint_maxvelc((1.0, 1.0, 1.0, 1.0, 1.0, 1.0))

                # # 设置末端运动最大线加速度 m/s^2
                # line_maxacc = 0.5
                # robot.set_end_max_line_acc(line_maxacc)

                # # 设置末端运动最大线速度 m/s
                # line_maxvelc = 0.2
                # robot.set_end_max_line_velc(line_maxvelc)

                # joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
                # logger.info("move joint to {0}".format(joint_radian))

                # robot.move_joint(joint_radian)

                # # 获取关节最大加速度
                # logger.info(robot.get_joint_maxacc())

                # # 正解测试
                # fk_ret = robot.forward_kin((-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008))
                # logger.info(fk_ret)

                # # 逆解
                joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                ik_result = robot.inverse_kin(joint_radian, fk_ret['pos'], fk_ret['ori'])
                logger.info(ik_result)

                # # 轴动1
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                # logger.info("move joint to {0}".format(joint_radian))
                # robot.move_joint(joint_radian)

                # # 轴动2
                # joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
                # logger.info("move joint to {0}".format(joint_radian))
                # robot.move_joint(joint_radian)

                # # 轴动3
                # joint_radian = (-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008)
                # logger.info("move joint to {0}".format(joint_radian))
                # robot.move_joint(joint_radian)

                # # 设置机械臂末端最大线加速度(m/s)
                # robot.set_end_max_line_acc(0.5)

                # # 获取机械臂末端最大线加速度(m/s)
                # robot.set_end_max_line_velc(0.2)

                # # 清除所有已经设置的全局路点
                # robot.remove_all_waypoint()

                # # 添加全局路点1,用于轨迹运动
                # joint_radian = (-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008)
                # robot.add_waypoint(joint_radian)

                # # 添加全局路点2,用于轨迹运动
                # joint_radian = (-0.211675, -0.325189, -1.466753, 0.429232, -1.570794, -0.211680)
                # robot.add_waypoint(joint_radian)

                # # 添加全局路点3,用于轨迹运动
                # joint_radian = (-0.037186, -0.224307, -1.398285, 0.396819, -1.570796, -0.037191)
                # robot.add_waypoint(joint_radian)

                # # 设置圆运动圈数
                # robot.set_circular_loop_times(3)

                # # 圆弧运动
                # logger.info("move_track ARC_CIR")
                # robot.move_track(RobotMoveTrackType.ARC_CIR)

                # # 清除所有已经设置的全局路点
                # robot.remove_all_waypoint()

                # # 机械臂轴动 回到0位
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                # logger.info("move joint to {0}".format(joint_radian))
                # robot.move_joint(joint_radian)

            # 断开服务器链接
            robot.disconnect()

    except RobotError as e:
        logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))


    finally:
        # 断开服务器链接
        if robot.connected:
            # 关闭机械臂 断电
            # robot.robot_shutdown()
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))

# 测试函数
def test_gripper():

    # 初始化logger
    logger_init()

    # 启动测试
    logger.info("{0} test gripper beginning...".format(Auboi5Robot.get_local_time()))

    # 系统初始化
    Auboi5Robot.initialize()

    # 创建机械臂控制类
    robot = Auboi5Robot()

    # 创建上下文
    handle = robot.create_context()

    # 打印上下文
    logger.info("robot.rshd={0}".format(handle))

    try:

        # 链接服务器
        ip = 'localhost'
        ip = '192.168.26.103'

        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:

            # # 设置碰撞等级
            robot.set_collision_class(7)


            robot.set_gripper_init() # 初始化夹具
            time.sleep(2) 
            robot.set_gripper_close() # 夹具闭合
            time.sleep(5)
            robot.set_gripper_open() # 夹具打开



            # # 获取 IO 状态
            # io_type = 5
            # io_name = "U_DO_01"
            # board_io_status = robot.get_board_io_status(io_type, io_name)
            # logger.info("获取 IO 状态：{0} ".format(board_io_status))



            # 断开服务器链接
            robot.disconnect()

    except RobotError as e:
        logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))


    finally:
        # 断开服务器链接
        if robot.connected:
            # 关闭机械臂 断电
            # robot.robot_shutdown()
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))

def step_test():
    # 初始化logger
    logger_init()

    # 启动测试
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # 系统初始化
    Auboi5Robot.initialize()

    # 创建机械臂控制类
    robot = Auboi5Robot()

    # 创建上下文
    handle = robot.create_context()

    # 打印上下文
    logger.info("robot.rshd={0}".format(handle))

    try:

        # 链接服务器
        ip = 'localhost'
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            # 重新上电
            robot.robot_shutdown()

            # 上电
            robot.robot_startup()

            # 设置碰撞等级
            robot.set_collision_class(7)

            # # 初始化全局配置文件
            # robot.init_profile()
            #
            # # logger.info(robot.get_board_io_config(RobotIOType.User_DI))
            #
            # # 获取当前位置
            # logger.info(robot.get_current_waypoint())
            #
            # joint_radian = (0, 0, 0, 0, 0, 0)
            # # 轴动到初始位置
            # robot.move_joint(joint_radian)
            #
            # # 沿Ｚ轴运动0.1毫米
            # current_pos = robot.get_current_waypoint()
            #
            # current_pos['pos'][2] -= 0.001
            #
            # ik_result = robot.inverse_kin(current_pos['joint'], current_pos['pos'], current_pos['ori'])
            # logger.info(ik_result)
            #
            # # joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
            # # logger.info("move joint to {0}".format(joint_radian))
            # # robot.move_joint(joint_radian)
            #
            # robot.move_line(ik_result['joint'])

            # 断开服务器链接
            robot.disconnect()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))

    finally:
        # 断开服务器链接
        if robot.connected:
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))


def move_rotate_test():
    # 初始化logger
    logger_init()

    # 启动测试
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # 系统初始化
    Auboi5Robot.initialize()

    # 创建机械臂控制类
    robot = Auboi5Robot()

    # 创建上下文
    handle = robot.create_context()

    # 打印上下文
    logger.info("robot.rshd={0}".format(handle))

    try:

        # 链接服务器
        ip = 'localhost'
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:

            # 重新上电
            # robot.robot_shutdown()

            # 上电
            # robot.robot_startup()

            # 设置碰撞等级
            # robot.set_collision_class(7)

            # joint_radian = (1, 0, 0, 0, 0, 0)
            # # 轴动到初始位置
            # robot.move_joint(joint_radian)

            joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
            logger.info("move joint to {0}".format(joint_radian))
            robot.move_joint(joint_radian)

            # 获取当前位置
            current_pos = robot.get_current_waypoint()

            # 工具转轴的向量（相对于法兰盘，这样需要测量得到x,y,z本测试样例默认以x=0,y=0,ｚ轴为0.1米）
            tool_pos_on_end = (0, 0, 0.10)

            # 工具姿态（w,x,y,z 相对于法兰盘，不知道的情况下，默认填写如下信息）
            tool_ori_on_end = (1, 0, 0, 0)

            tool_desc = {"pos": tool_pos_on_end, "ori": tool_ori_on_end}

            # 得到法兰盘工具末端点相对于基座坐标系中的位置
            tool_pos_on_base = robot.base_to_base_additional_tool(current_pos['pos'],
                                                                  current_pos['ori'],
                                                                  tool_desc)

            logger.info("current_pos={0}".format(current_pos['pos'][0]))

            logger.info("tool_pos_on_base={0}".format(tool_pos_on_base['pos'][0]))

            # 讲工具转轴向量平移到基座坐标系下(旋转方向符合右手准则)
            rotate_axis = map(lambda a, b: a - b, tool_pos_on_base['pos'], current_pos['pos'])

            logger.info("rotate_axis={0}".format(rotate_axis))

            # 坐标系默认使用基座坐标系（默认填写下面的值就可以了）
            user_coord = {'coord_type': RobotCoordType.Robot_Base_Coordinate,
                          'calibrate_method': 0,
                          'calibrate_points':
                              {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                               "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                               "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
                          'tool_desc':
                              {"pos": (0.0, 0.0, 0.0),
                               "ori": (1.0, 0.0, 0.0, 0.0)}
                          }

            # 调用转轴旋转接口，最后一个参数为旋转角度（弧度）
            robot.move_rotate(user_coord, rotate_axis, 1)

            # 断开服务器链接
            robot.disconnect()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))

    finally:
        # 断开服务器链接
        if robot.connected:
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()


def test_rsm():
    # 初始化logger
    logger_init()

    # 启动测试
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # 系统初始化
    Auboi5Robot.initialize()

    # 创建机械臂控制类
    robot = Auboi5Robot()

    # 创建上下文
    handle = robot.create_context()

    # 打印上下文
    logger.info("robot.rshd={0}".format(handle))

    try:

        # 链接服务器
        #ip = 'localhost'
        ip = '192.168.10.88'
        port = 8899
        result = robot.connect(ip, port)
        
        #robot.enable_robot_event()

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:

            # robot.move_pause()

            #joint_radian = (0, 0, 0, 0, 0, 0)
            # 轴动到初始位置
            #robot.move_joint(joint_radian)

            while True:
                time.sleep(0.05)

                rel = robot.set_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_02, 0)
                print(rel)
                print("++++++++++++++++++++++++")
                #result = robot.get_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_02)
                #print(result)
                # print("*********************************")

                time.sleep(2)
                # rel1 = robot.set_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_02, 0)
                # print(rel1)
                # print("++++++++++++++++++++++++")

            # 断开服务器链接
            robot.disconnect()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))


    finally:
        # 断开服务器链接
        if robot.connected:
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()


def runWaypoint(queue):
    while True:
        # while not queue.empty():
        print(queue.get(True))


def test_process_demo():

    # 初始化logger
    logger_init()

    # 启动测试
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # 系统初始化
    Auboi5Robot.initialize()

    # 创建机械臂控制类
    robot = Auboi5Robot()

    # 创建上下文
    handle = robot.create_context()

    # 打印上下文
    logger.info("robot.rshd={0}".format(handle))

    try:

        # time.sleep(0.2)
        # process_get_robot_current_status = GetRobotWaypointProcess()
        # process_get_robot_current_status.daemon = True
        # process_get_robot_current_status.start()
        # time.sleep(0.2)

        queue = Queue()

        p = Process(target=runWaypoint, args=(queue,))
        p.start()
        time.sleep(5)
        print("process started.")

        # 链接服务器
        #ip = 'localhost'
        ip = '192.168.65.131'
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            robot.project_startup()
            robot.enable_robot_event()
            robot.init_profile()
            joint_maxvelc = (2.596177, 2.596177, 2.596177, 3.110177, 3.110177, 3.110177)
            joint_maxacc = (17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5)
            robot.set_joint_maxacc(joint_maxacc)
            robot.set_joint_maxvelc(joint_maxvelc)
            robot.set_arrival_ahead_blend(0.05)
            while True:
                time.sleep(1)

                joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
                robot.move_joint(joint_radian, True)
                

                joint_radian = (55.5/180.0*pi, -20.5/180.0*pi, -72.5/180.0*pi, 38.5/180.0*pi, -90.5/180.0*pi, 55.5/180.0*pi)
                robot.move_joint(joint_radian, True)

                joint_radian = (0, 0, 0, 0, 0, 0)
                robot.move_joint(joint_radian, True)

                print("-----------------------------")

                queue.put(joint_radian)

                robot.project_stop()

                # time.sleep(5)

                # process_get_robot_current_status.test()

                # print("-----------------------------")

                # 断开服务器链接
            robot.disconnect()

    except KeyboardInterrupt:
        robot.move_stop()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))



    finally:
        # 断开服务器链接
        if robot.connected:
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        print("run end-------------------------")


if __name__ == '__main__':

    test_gripper()

    logger.info("test completed")