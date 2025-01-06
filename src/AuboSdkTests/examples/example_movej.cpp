#include "example_movej.h"
#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>

#define SERVER_HOST "192.168.1.100"
#define SERVER_PORT 8899

Example_MoveJ::Example_MoveJ()
{

}

void Example_MoveJ::movej()
{
    ServiceInterface robotService;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "登录成功" << std::endl;
    }
    else
    {
        std::cerr << "登录失败" << std::endl;
    }

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr << "机械臂初始化成功." << std::endl;
    }
    else
    {
        std::cerr << "机械臂初始化失败." << std::endl;
    }

    //初始化运动属性
    robotService.robotServiceInitGlobalMoveProfile();

    //设置关节运动最大加速度
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 30*M_PI/180;
    jointMaxAcc.jointPara[1] = 30*M_PI/180;
    jointMaxAcc.jointPara[2] = 30*M_PI/180;
    jointMaxAcc.jointPara[3] = 30*M_PI/180;
    jointMaxAcc.jointPara[4] = 30*M_PI/180;
    jointMaxAcc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    //设置关节运动最大速度
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 30*M_PI/180;
    jointMaxVelc.jointPara[1] = 30*M_PI/180;
    jointMaxVelc.jointPara[2] = 30*M_PI/180;
    jointMaxVelc.jointPara[3] = 30*M_PI/180;
    jointMaxVelc.jointPara[4] = 30*M_PI/180;
    jointMaxVelc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    //关节角 jointAngle
    double jointAngle[6] = {};
    jointAngle[0] = 173.108713*M_PI/180;
    jointAngle[1] = -12.075005*M_PI/180;
    jointAngle[2] = -83.663342*M_PI/180;
    jointAngle[3] = -15.641249*M_PI/180;
    jointAngle[4] = -89.140000*M_PI/180;
    jointAngle[5] = -28.328713*M_PI/180;

    //关节运动
    robotService.robotServiceJointMove(jointAngle, true);

}

void Example_MoveJ::movejToTargetPositionByRelative1()
{
    ServiceInterface robotService;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "登录成功" << std::endl;
    }
    else
    {
        std::cerr << "登录失败" << std::endl;
    }

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr << "机械臂初始化成功." << std::endl;
    }
    else
    {
        std::cerr << "机械臂初始化失败." << std::endl;
    }

    //初始化运动属性
    robotService.robotServiceInitGlobalMoveProfile();

    //设置关节运动最大加速度
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 30*M_PI/180;
    jointMaxAcc.jointPara[1] = 30*M_PI/180;
    jointMaxAcc.jointPara[2] = 30*M_PI/180;
    jointMaxAcc.jointPara[3] = 30*M_PI/180;
    jointMaxAcc.jointPara[4] = 30*M_PI/180;
    jointMaxAcc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    //设置关节运动最大速度
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 30*M_PI/180;
    jointMaxVelc.jointPara[1] = 30*M_PI/180;
    jointMaxVelc.jointPara[2] = 30*M_PI/180;
    jointMaxVelc.jointPara[3] = 30*M_PI/180;
    jointMaxVelc.jointPara[4] = 30*M_PI/180;
    jointMaxVelc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    //起始路点
    double initAngle[6] = {};
    initAngle[0] = 173.108713*M_PI/180;
    initAngle[1] = -12.075005*M_PI/180;
    initAngle[2] = -83.663342*M_PI/180;
    initAngle[3] = -15.641249*M_PI/180;
    initAngle[4] = -89.140000*M_PI/180;
    initAngle[5] = -28.328713*M_PI/180;

    //关节运动到起始路点
    robotService.robotServiceJointMove(initAngle, true);

    //设置基坐标系
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool baseCoord;
    baseCoord.coordType = aubo_robot_namespace::BaseCoordinate;

    //设置偏移量
    aubo_robot_namespace::MoveRelative moveRelative;
    moveRelative.relativePosition[0] = 0.001;
    moveRelative.relativePosition[1] = 0;
    moveRelative.relativePosition[2] = 0;

    //关节运动到目标位置，其中目标位置是法兰盘中心在基坐标系下通过相对当前位置沿X轴偏移0.001m给出的
    robotService.robotMoveJointToTargetPositionByRelative(baseCoord, moveRelative, true);

}

void Example_MoveJ::movejToTargetPositionByRelative2()
{
    ServiceInterface robotService;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "登录成功" << std::endl;
    }
    else
    {
        std::cerr << "登录失败" << std::endl;
    }

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr << "机械臂初始化成功." << std::endl;
    }
    else
    {
        std::cerr << "机械臂初始化失败." << std::endl;
    }

    //初始化运动属性
    robotService.robotServiceInitGlobalMoveProfile();

    //设置关节运动最大加速度
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 30*M_PI/180;
    jointMaxAcc.jointPara[1] = 30*M_PI/180;
    jointMaxAcc.jointPara[2] = 30*M_PI/180;
    jointMaxAcc.jointPara[3] = 30*M_PI/180;
    jointMaxAcc.jointPara[4] = 30*M_PI/180;
    jointMaxAcc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    //设置关节运动最大速度
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 30*M_PI/180;
    jointMaxVelc.jointPara[1] = 30*M_PI/180;
    jointMaxVelc.jointPara[2] = 30*M_PI/180;
    jointMaxVelc.jointPara[3] = 30*M_PI/180;
    jointMaxVelc.jointPara[4] = 30*M_PI/180;
    jointMaxVelc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    //起始路点
    double initAngle[6] = {};
    initAngle[0] = 173.108713*M_PI/180;
    initAngle[1] = -12.075005*M_PI/180;
    initAngle[2] = -83.663342*M_PI/180;
    initAngle[3] = -15.641249*M_PI/180;
    initAngle[4] = -89.140000*M_PI/180;
    initAngle[5] = -28.328713*M_PI/180;

    //关节运动到起始路点
    robotService.robotServiceJointMove(initAngle, true);

    //设置用户坐标系
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;
    userCoord.coordType = aubo_robot_namespace::WorldCoordinate;
    userCoord.methods = aubo_robot_namespace::Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOYPlane;

    userCoord.wayPointArray[0].jointPos[0] = -75.093279*M_PI/180;
    userCoord.wayPointArray[0].jointPos[1] = 28.544643*M_PI/180;
    userCoord.wayPointArray[0].jointPos[2] = -114.313905*M_PI/180;
    userCoord.wayPointArray[0].jointPos[3] = -62.769247*M_PI/180;
    userCoord.wayPointArray[0].jointPos[4] = -87.343517*M_PI/180;
    userCoord.wayPointArray[0].jointPos[5] = -27.888262*M_PI/180;

    userCoord.wayPointArray[1].jointPos[0] = -89.239837*M_PI/180;
    userCoord.wayPointArray[1].jointPos[1] = 23.936171*M_PI/180;
    userCoord.wayPointArray[1].jointPos[2] = -122.299277*M_PI/180;
    userCoord.wayPointArray[1].jointPos[3] = -65.208902*M_PI/180;
    userCoord.wayPointArray[1].jointPos[4] = -85.011123*M_PI/180;
    userCoord.wayPointArray[1].jointPos[5] = -41.87417*M_PI/180;

    userCoord.wayPointArray[2].jointPos[0] = -77.059212*M_PI/180;
    userCoord.wayPointArray[2].jointPos[1] = 35.509518*M_PI/180;
    userCoord.wayPointArray[2].jointPos[2] = -101.108547*M_PI/180;
    userCoord.wayPointArray[2].jointPos[3] = -56.433133*M_PI/180;
    userCoord.wayPointArray[2].jointPos[4] = -87.006734*M_PI/180;
    userCoord.wayPointArray[2].jointPos[5] = -29.827440*M_PI/180;

    aubo_robot_namespace::ToolInEndDesc toolUserCoord;
    toolUserCoord.toolInEndPosition.x =  -0.177341;
    toolUserCoord.toolInEndPosition.y = 0.002327;
    toolUserCoord.toolInEndPosition.z = 0.146822;
    toolUserCoord.toolInEndOrientation.w = 1;
    toolUserCoord.toolInEndOrientation.x = 0;
    toolUserCoord.toolInEndOrientation.y = 0;
    toolUserCoord.toolInEndOrientation.z = 0;
    userCoord.toolDesc = toolUserCoord;


    //设置偏移量
    aubo_robot_namespace::MoveRelative moveRelative;
    moveRelative.relativePosition[0] = 0.001;
    moveRelative.relativePosition[1] = 0;
    moveRelative.relativePosition[2] = 0;

    //关节运动到目标位置，其中目标位置是法兰盘中心在用户坐标系下通过相对当前位置沿X轴偏移0.001m给出的
    robotService.robotMoveJointToTargetPositionByRelative(userCoord, moveRelative, true);

}

void Example_MoveJ::movejToTargetPositionByRelative3()
{
    ServiceInterface robotService;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "登录成功" << std::endl;
    }
    else
    {
        std::cerr << "登录失败" << std::endl;
    }

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr << "机械臂初始化成功." << std::endl;
    }
    else
    {
        std::cerr << "机械臂初始化失败." << std::endl;
    }

    //初始化运动属性
    robotService.robotServiceInitGlobalMoveProfile();

    //设置关节运动最大加速度
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 30*M_PI/180;
    jointMaxAcc.jointPara[1] = 30*M_PI/180;
    jointMaxAcc.jointPara[2] = 30*M_PI/180;
    jointMaxAcc.jointPara[3] = 30*M_PI/180;
    jointMaxAcc.jointPara[4] = 30*M_PI/180;
    jointMaxAcc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    //设置关节运动最大速度
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 30*M_PI/180;
    jointMaxVelc.jointPara[1] = 30*M_PI/180;
    jointMaxVelc.jointPara[2] = 30*M_PI/180;
    jointMaxVelc.jointPara[3] = 30*M_PI/180;
    jointMaxVelc.jointPara[4] = 30*M_PI/180;
    jointMaxVelc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    //起始路点
    double initAngle[6] = {};
    initAngle[0] = 173.108713*M_PI/180;
    initAngle[1] = -12.075005*M_PI/180;
    initAngle[2] = -83.663342*M_PI/180;
    initAngle[3] = -15.641249*M_PI/180;
    initAngle[4] = -89.140000*M_PI/180;
    initAngle[5] = -28.328713*M_PI/180;

    //关节运动到起始路点
    robotService.robotServiceJointMove(initAngle, true);

    //设置工具
    aubo_robot_namespace::ToolInEndDesc toolEnd;
    toolEnd.toolInEndPosition.x =  -0.177341;
    toolEnd.toolInEndPosition.y = 0.002327;
    toolEnd.toolInEndPosition.z = 0.146822;
    toolEnd.toolInEndOrientation.w = 1;
    toolEnd.toolInEndOrientation.x = 0;
    toolEnd.toolInEndOrientation.y = 0;
    toolEnd.toolInEndOrientation.z = 0;
    robotService.robotServiceSetToolKinematicsParam(toolEnd);

    //设置工具坐标系
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool endCoord;
    endCoord.coordType = aubo_robot_namespace::EndCoordinate;
    endCoord.toolDesc = toolEnd;

    //设置偏移量
    aubo_robot_namespace::MoveRelative moveRelative;
    moveRelative.relativePosition[0] = 0.001;
    moveRelative.relativePosition[1] = 0;
    moveRelative.relativePosition[2] = 0;

    //关节运动到目标位置，其中目标位置是工具末端在工具坐标系下通过相对当前位置沿X轴偏移0.001m给出的
    robotService.robotMoveJointToTargetPositionByRelative(endCoord, moveRelative, true);

}

void Example_MoveJ::movejToTargetPositionByRelative4()
{
    ServiceInterface robotService;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "登录成功" << std::endl;
    }
    else
    {
        std::cerr << "登录失败" << std::endl;
    }

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr << "机械臂初始化成功." << std::endl;
    }
    else
    {
        std::cerr << "机械臂初始化失败." << std::endl;
    }

    //初始化运动属性
    robotService.robotServiceInitGlobalMoveProfile();

    //设置关节运动最大加速度
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 30*M_PI/180;
    jointMaxAcc.jointPara[1] = 30*M_PI/180;
    jointMaxAcc.jointPara[2] = 30*M_PI/180;
    jointMaxAcc.jointPara[3] = 30*M_PI/180;
    jointMaxAcc.jointPara[4] = 30*M_PI/180;
    jointMaxAcc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    //设置关节运动最大速度
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 30*M_PI/180;
    jointMaxVelc.jointPara[1] = 30*M_PI/180;
    jointMaxVelc.jointPara[2] = 30*M_PI/180;
    jointMaxVelc.jointPara[3] = 30*M_PI/180;
    jointMaxVelc.jointPara[4] = 30*M_PI/180;
    jointMaxVelc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    //起始路点
    double initAngle[6] = {};
    initAngle[0] = 173.108713*M_PI/180;
    initAngle[1] = -12.075005*M_PI/180;
    initAngle[2] = -83.663342*M_PI/180;
    initAngle[3] = -15.641249*M_PI/180;
    initAngle[4] = -89.140000*M_PI/180;
    initAngle[5] = -28.328713*M_PI/180;

    //关节运动到起始路点
    robotService.robotServiceJointMove(initAngle, true);

    //设置工具
    aubo_robot_namespace::ToolInEndDesc toolEnd;
    toolEnd.toolInEndPosition.x =  -0.177341;
    toolEnd.toolInEndPosition.y = 0.002327;
    toolEnd.toolInEndPosition.z = 0.146822;
    toolEnd.toolInEndOrientation.w = 1;
    toolEnd.toolInEndOrientation.x = 0;
    toolEnd.toolInEndOrientation.y = 0;
    toolEnd.toolInEndOrientation.z = 0;
    robotService.robotServiceSetToolKinematicsParam(toolEnd);

    //设置基坐标系
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool baseCoord;
    baseCoord.coordType = aubo_robot_namespace::BaseCoordinate;

    //设置偏移量
    aubo_robot_namespace::MoveRelative moveRelative;
    moveRelative.relativePosition[0] = 0.001;
    moveRelative.relativePosition[1] = 0;
    moveRelative.relativePosition[2] = 0;

    //关节运动到目标位置，其中目标位置是工具末端在基坐标系下通过相对当前位置沿X轴偏移0.001m给出的
    robotService.robotMoveJointToTargetPositionByRelative(baseCoord, moveRelative, true);
}

void Example_MoveJ::movejToTargetPositionByRelative5()
{
    ServiceInterface robotService;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "登录成功" << std::endl;
    }
    else
    {
        std::cerr << "登录失败" << std::endl;
    }

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr << "机械臂初始化成功." << std::endl;
    }
    else
    {
        std::cerr << "机械臂初始化失败." << std::endl;
    }

    //初始化运动属性
    robotService.robotServiceInitGlobalMoveProfile();

    //设置关节运动最大加速度
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 30*M_PI/180;
    jointMaxAcc.jointPara[1] = 30*M_PI/180;
    jointMaxAcc.jointPara[2] = 30*M_PI/180;
    jointMaxAcc.jointPara[3] = 30*M_PI/180;
    jointMaxAcc.jointPara[4] = 30*M_PI/180;
    jointMaxAcc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    //设置关节运动最大速度
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 30*M_PI/180;
    jointMaxVelc.jointPara[1] = 30*M_PI/180;
    jointMaxVelc.jointPara[2] = 30*M_PI/180;
    jointMaxVelc.jointPara[3] = 30*M_PI/180;
    jointMaxVelc.jointPara[4] = 30*M_PI/180;
    jointMaxVelc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    //起始路点
    double initAngle[6] = {};
    initAngle[0] = 173.108713*M_PI/180;
    initAngle[1] = -12.075005*M_PI/180;
    initAngle[2] = -83.663342*M_PI/180;
    initAngle[3] = -15.641249*M_PI/180;
    initAngle[4] = -89.140000*M_PI/180;
    initAngle[5] = -28.328713*M_PI/180;

    //关节运动到起始路点
    robotService.robotServiceJointMove(initAngle, true);

    //设置工具
    aubo_robot_namespace::ToolInEndDesc toolEnd;
    toolEnd.toolInEndPosition.x =  -0.177341;
    toolEnd.toolInEndPosition.y = 0.002327;
    toolEnd.toolInEndPosition.z = 0.146822;
    toolEnd.toolInEndOrientation.w = 1;
    toolEnd.toolInEndOrientation.x = 0;
    toolEnd.toolInEndOrientation.y = 0;
    toolEnd.toolInEndOrientation.z = 0;
    robotService.robotServiceSetToolKinematicsParam(toolEnd);

    //设置用户坐标系
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;
    userCoord.coordType = aubo_robot_namespace::WorldCoordinate;
    userCoord.methods = aubo_robot_namespace::Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOYPlane;

    userCoord.wayPointArray[0].jointPos[0] = -75.093279*M_PI/180;
    userCoord.wayPointArray[0].jointPos[1] = 28.544643*M_PI/180;
    userCoord.wayPointArray[0].jointPos[2] = -114.313905*M_PI/180;
    userCoord.wayPointArray[0].jointPos[3] = -62.769247*M_PI/180;
    userCoord.wayPointArray[0].jointPos[4] = -87.343517*M_PI/180;
    userCoord.wayPointArray[0].jointPos[5] = -27.888262*M_PI/180;

    userCoord.wayPointArray[1].jointPos[0] = -89.239837*M_PI/180;
    userCoord.wayPointArray[1].jointPos[1] = 23.936171*M_PI/180;
    userCoord.wayPointArray[1].jointPos[2] = -122.299277*M_PI/180;
    userCoord.wayPointArray[1].jointPos[3] = -65.208902*M_PI/180;
    userCoord.wayPointArray[1].jointPos[4] = -85.011123*M_PI/180;
    userCoord.wayPointArray[1].jointPos[5] = -41.87417*M_PI/180;

    userCoord.wayPointArray[2].jointPos[0] = -77.059212*M_PI/180;
    userCoord.wayPointArray[2].jointPos[1] = 35.509518*M_PI/180;
    userCoord.wayPointArray[2].jointPos[2] = -101.108547*M_PI/180;
    userCoord.wayPointArray[2].jointPos[3] = -56.433133*M_PI/180;
    userCoord.wayPointArray[2].jointPos[4] = -87.006734*M_PI/180;
    userCoord.wayPointArray[2].jointPos[5] = -29.827440*M_PI/180;

    userCoord.toolDesc = toolEnd;

    //设置偏移量
    aubo_robot_namespace::MoveRelative moveRelative;
    moveRelative.relativePosition[0] = 0.001;
    moveRelative.relativePosition[1] = 0;
    moveRelative.relativePosition[2] = 0;

    //关节运动到目标位置，其中目标位置是工具末端在用户坐标系下通过相对当前位置沿X轴偏移0.001m给出的
    robotService.robotMoveJointToTargetPositionByRelative(userCoord, moveRelative, true);

}

void Example_MoveJ::movejToTargetPosition1()
{
    ServiceInterface robotService;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "登录成功" << std::endl;
    }
    else
    {
        std::cerr << "登录失败" << std::endl;
    }

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr << "机械臂初始化成功." << std::endl;
    }
    else
    {
        std::cerr << "机械臂初始化失败." << std::endl;
    }

    //初始化运动属性
    robotService.robotServiceInitGlobalMoveProfile();

    //设置关节运动最大加速度
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 30*M_PI/180;
    jointMaxAcc.jointPara[1] = 30*M_PI/180;
    jointMaxAcc.jointPara[2] = 30*M_PI/180;
    jointMaxAcc.jointPara[3] = 30*M_PI/180;
    jointMaxAcc.jointPara[4] = 30*M_PI/180;
    jointMaxAcc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    //设置关节运动最大速度
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 30*M_PI/180;
    jointMaxVelc.jointPara[1] = 30*M_PI/180;
    jointMaxVelc.jointPara[2] = 30*M_PI/180;
    jointMaxVelc.jointPara[3] = 30*M_PI/180;
    jointMaxVelc.jointPara[4] = 30*M_PI/180;
    jointMaxVelc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    //起始路点
    double initAngle[6] = {};
    initAngle[0] = 173.108713*M_PI/180;
    initAngle[1] = -12.075005*M_PI/180;
    initAngle[2] = -83.663342*M_PI/180;
    initAngle[3] = -15.641249*M_PI/180;
    initAngle[4] = -89.140000*M_PI/180;
    initAngle[5] = -28.328713*M_PI/180;

    //关节运动到起始路点
    robotService.robotServiceJointMove(initAngle, true);

    //设置工具参数，为法兰盘中心
    aubo_robot_namespace::ToolInEndDesc toolEnd;
    toolEnd.toolInEndPosition.x = 0;
    toolEnd.toolInEndPosition.y = 0;
    toolEnd.toolInEndPosition.z = 0;
    toolEnd.toolInEndOrientation.w = 1;
    toolEnd.toolInEndOrientation.x = 0;
    toolEnd.toolInEndOrientation.y = 0;
    toolEnd.toolInEndOrientation.z = 0;

    //设置基坐标系
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool baseCoord;
    baseCoord.coordType = aubo_robot_namespace::BaseCoordinate;

    //设置目标位置，法兰盘中心在基坐标系下的位置
    aubo_robot_namespace::Pos posOnBase;
    posOnBase.x = -0.2;
    posOnBase.y = -0.6;
    posOnBase.z = 0;

    //轴动运动至目标位置
    robotService.robotMoveJointToTargetPosition(baseCoord, posOnBase, toolEnd, true);


}

void Example_MoveJ::movejToTargetPosition2()
{
    ServiceInterface robotService;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "登录成功" << std::endl;
    }
    else
    {
        std::cerr << "登录失败" << std::endl;
    }

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr << "机械臂初始化成功." << std::endl;
    }
    else
    {
        std::cerr << "机械臂初始化失败." << std::endl;
    }

    //初始化运动属性
    robotService.robotServiceInitGlobalMoveProfile();

    //设置关节运动最大加速度
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 30*M_PI/180;
    jointMaxAcc.jointPara[1] = 30*M_PI/180;
    jointMaxAcc.jointPara[2] = 30*M_PI/180;
    jointMaxAcc.jointPara[3] = 30*M_PI/180;
    jointMaxAcc.jointPara[4] = 30*M_PI/180;
    jointMaxAcc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    //设置关节运动最大速度
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 30*M_PI/180;
    jointMaxVelc.jointPara[1] = 30*M_PI/180;
    jointMaxVelc.jointPara[2] = 30*M_PI/180;
    jointMaxVelc.jointPara[3] = 30*M_PI/180;
    jointMaxVelc.jointPara[4] = 30*M_PI/180;
    jointMaxVelc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    //起始路点
    double initAngle[6] = {};
    initAngle[0] = 173.108713*M_PI/180;
    initAngle[1] = -12.075005*M_PI/180;
    initAngle[2] = -83.663342*M_PI/180;
    initAngle[3] = -15.641249*M_PI/180;
    initAngle[4] = -89.140000*M_PI/180;
    initAngle[5] = -28.328713*M_PI/180;

    //关节运动到起始路点
    robotService.robotServiceJointMove(initAngle, true);

    //设置工具参数，为法兰盘中心
    aubo_robot_namespace::ToolInEndDesc toolEnd;
    toolEnd.toolInEndPosition.x = 0;
    toolEnd.toolInEndPosition.y = 0;
    toolEnd.toolInEndPosition.z = 0;
    toolEnd.toolInEndOrientation.w = 1;
    toolEnd.toolInEndOrientation.x = 0;
    toolEnd.toolInEndOrientation.y = 0;
    toolEnd.toolInEndOrientation.z = 0;

    //设置用户坐标系
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;
    userCoord.coordType = aubo_robot_namespace::WorldCoordinate;
    userCoord.methods = aubo_robot_namespace::Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOYPlane;

    userCoord.wayPointArray[0].jointPos[0] = -75.093279*M_PI/180;
    userCoord.wayPointArray[0].jointPos[1] = 28.544643*M_PI/180;
    userCoord.wayPointArray[0].jointPos[2] = -114.313905*M_PI/180;
    userCoord.wayPointArray[0].jointPos[3] = -62.769247*M_PI/180;
    userCoord.wayPointArray[0].jointPos[4] = -87.343517*M_PI/180;
    userCoord.wayPointArray[0].jointPos[5] = -27.888262*M_PI/180;

    userCoord.wayPointArray[1].jointPos[0] = -89.239837*M_PI/180;
    userCoord.wayPointArray[1].jointPos[1] = 23.936171*M_PI/180;
    userCoord.wayPointArray[1].jointPos[2] = -122.299277*M_PI/180;
    userCoord.wayPointArray[1].jointPos[3] = -65.208902*M_PI/180;
    userCoord.wayPointArray[1].jointPos[4] = -85.011123*M_PI/180;
    userCoord.wayPointArray[1].jointPos[5] = -41.87417*M_PI/180;

    userCoord.wayPointArray[2].jointPos[0] = -77.059212*M_PI/180;
    userCoord.wayPointArray[2].jointPos[1] = 35.509518*M_PI/180;
    userCoord.wayPointArray[2].jointPos[2] = -101.108547*M_PI/180;
    userCoord.wayPointArray[2].jointPos[3] = -56.433133*M_PI/180;
    userCoord.wayPointArray[2].jointPos[4] = -87.006734*M_PI/180;
    userCoord.wayPointArray[2].jointPos[5] = -29.827440*M_PI/180;

    //坐标系的工具参数
    aubo_robot_namespace::ToolInEndDesc toolDesc;
    toolDesc.toolInEndPosition.x =  -0.177341;
    toolDesc.toolInEndPosition.y = 0.002327;
    toolDesc.toolInEndPosition.z = 0.146822;
    toolDesc.toolInEndOrientation.w = 1;
    toolDesc.toolInEndOrientation.x = 0;
    toolDesc.toolInEndOrientation.y = 0;
    toolDesc.toolInEndOrientation.z = 0;
    userCoord.toolDesc = toolDesc;

    //设置目标位置，法兰盘中心在用户坐标系下的位置
    aubo_robot_namespace::Pos posOnUser;
    posOnUser.x = -0.220305;
    posOnUser.y = -1.152177;
    posOnUser.z = 0.216184;

    //轴动运动至目标位置
    robotService.robotMoveJointToTargetPosition(userCoord, posOnUser, toolEnd, true);

}

void Example_MoveJ::movejToTargetPosition3()
{   
    ServiceInterface robotService;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "登录成功" << std::endl;
    }
    else
    {
        std::cerr << "登录失败" << std::endl;
    }

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr << "机械臂初始化成功." << std::endl;
    }
    else
    {
        std::cerr << "机械臂初始化失败." << std::endl;
    }

    //初始化运动属性
    robotService.robotServiceInitGlobalMoveProfile();

    //设置关节运动最大加速度
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 30*M_PI/180;
    jointMaxAcc.jointPara[1] = 30*M_PI/180;
    jointMaxAcc.jointPara[2] = 30*M_PI/180;
    jointMaxAcc.jointPara[3] = 30*M_PI/180;
    jointMaxAcc.jointPara[4] = 30*M_PI/180;
    jointMaxAcc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    //设置关节运动最大速度
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 30*M_PI/180;
    jointMaxVelc.jointPara[1] = 30*M_PI/180;
    jointMaxVelc.jointPara[2] = 30*M_PI/180;
    jointMaxVelc.jointPara[3] = 30*M_PI/180;
    jointMaxVelc.jointPara[4] = 30*M_PI/180;
    jointMaxVelc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    //起始路点
    double initAngle[6] = {};
    initAngle[0] = 173.108713*M_PI/180;
    initAngle[1] = -12.075005*M_PI/180;
    initAngle[2] = -83.663342*M_PI/180;
    initAngle[3] = -15.641249*M_PI/180;
    initAngle[4] = -89.140000*M_PI/180;
    initAngle[5] = -28.328713*M_PI/180;

    //关节运动到起始路点
    robotService.robotServiceJointMove(initAngle, true);

    //设置工具参数，为工具末端
    aubo_robot_namespace::ToolInEndDesc toolEnd;
    toolEnd.toolInEndPosition.x = -0.177341;
    toolEnd.toolInEndPosition.y = 0.002327;
    toolEnd.toolInEndPosition.z = 0.146822;
    toolEnd.toolInEndOrientation.w = 1;
    toolEnd.toolInEndOrientation.x = 0;
    toolEnd.toolInEndOrientation.y = 0;
    toolEnd.toolInEndOrientation.z = 0;

    //设置基坐标系
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool baseCoord;
    baseCoord.coordType = aubo_robot_namespace::BaseCoordinate;

    //设置目标位置，工具末端在基坐标系下的位置
    aubo_robot_namespace::Pos posOnBase;
    posOnBase.x = 0.6;
    posOnBase.y = 0.0;
    posOnBase.z = 0.5;

    //轴动运动至目标位置
    robotService.robotMoveJointToTargetPosition(baseCoord, posOnBase, toolEnd, true);
}

void Example_MoveJ::movejToTargetPosition4()
{
    ServiceInterface robotService;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "登录成功" << std::endl;
    }
    else
    {
        std::cerr << "登录失败" << std::endl;
    }

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr << "机械臂初始化成功." << std::endl;
    }
    else
    {
        std::cerr << "机械臂初始化失败." << std::endl;
    }

    //初始化运动属性
    robotService.robotServiceInitGlobalMoveProfile();

    //设置关节运动最大加速度
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 30*M_PI/180;
    jointMaxAcc.jointPara[1] = 30*M_PI/180;
    jointMaxAcc.jointPara[2] = 30*M_PI/180;
    jointMaxAcc.jointPara[3] = 30*M_PI/180;
    jointMaxAcc.jointPara[4] = 30*M_PI/180;
    jointMaxAcc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    //设置关节运动最大速度
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 30*M_PI/180;
    jointMaxVelc.jointPara[1] = 30*M_PI/180;
    jointMaxVelc.jointPara[2] = 30*M_PI/180;
    jointMaxVelc.jointPara[3] = 30*M_PI/180;
    jointMaxVelc.jointPara[4] = 30*M_PI/180;
    jointMaxVelc.jointPara[5] = 30*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    //起始路点
    double initAngle[6] = {};
    initAngle[0] = 173.108713*M_PI/180;
    initAngle[1] = -12.075005*M_PI/180;
    initAngle[2] = -83.663342*M_PI/180;
    initAngle[3] = -15.641249*M_PI/180;
    initAngle[4] = -89.140000*M_PI/180;
    initAngle[5] = -28.328713*M_PI/180;

    //关节运动到起始路点
    robotService.robotServiceJointMove(initAngle, true);

    //设置工具参数，为工具末端
    aubo_robot_namespace::ToolInEndDesc toolEnd;
    toolEnd.toolInEndPosition.x = -0.177341;
    toolEnd.toolInEndPosition.y = 0.002327;
    toolEnd.toolInEndPosition.z = 0.146822;
    toolEnd.toolInEndOrientation.w = 1;
    toolEnd.toolInEndOrientation.x = 0;
    toolEnd.toolInEndOrientation.y = 0;
    toolEnd.toolInEndOrientation.z = 0;

    //设置用户坐标系
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;
    userCoord.coordType = aubo_robot_namespace::WorldCoordinate;
    userCoord.methods = aubo_robot_namespace::Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOYPlane;

    userCoord.wayPointArray[0].jointPos[0] = -75.093279*M_PI/180;
    userCoord.wayPointArray[0].jointPos[1] = 28.544643*M_PI/180;
    userCoord.wayPointArray[0].jointPos[2] = -114.313905*M_PI/180;
    userCoord.wayPointArray[0].jointPos[3] = -62.769247*M_PI/180;
    userCoord.wayPointArray[0].jointPos[4] = -87.343517*M_PI/180;
    userCoord.wayPointArray[0].jointPos[5] = -27.888262*M_PI/180;

    userCoord.wayPointArray[1].jointPos[0] = -89.239837*M_PI/180;
    userCoord.wayPointArray[1].jointPos[1] = 23.936171*M_PI/180;
    userCoord.wayPointArray[1].jointPos[2] = -122.299277*M_PI/180;
    userCoord.wayPointArray[1].jointPos[3] = -65.208902*M_PI/180;
    userCoord.wayPointArray[1].jointPos[4] = -85.011123*M_PI/180;
    userCoord.wayPointArray[1].jointPos[5] = -41.87417*M_PI/180;

    userCoord.wayPointArray[2].jointPos[0] = -77.059212*M_PI/180;
    userCoord.wayPointArray[2].jointPos[1] = 35.509518*M_PI/180;
    userCoord.wayPointArray[2].jointPos[2] = -101.108547*M_PI/180;
    userCoord.wayPointArray[2].jointPos[3] = -56.433133*M_PI/180;
    userCoord.wayPointArray[2].jointPos[4] = -87.006734*M_PI/180;
    userCoord.wayPointArray[2].jointPos[5] = -29.827440*M_PI/180;

    //坐标系的工具参数
    aubo_robot_namespace::ToolInEndDesc toolDesc;
    toolDesc.toolInEndPosition.x =  -0.177341;
    toolDesc.toolInEndPosition.y = 0.002327;
    toolDesc.toolInEndPosition.z = 0.146822;
    toolDesc.toolInEndOrientation.w = 1;
    toolDesc.toolInEndOrientation.x = 0;
    toolDesc.toolInEndOrientation.y = 0;
    toolDesc.toolInEndOrientation.z = 0;
    userCoord.toolDesc = toolDesc;

    //设置目标位置，工具末端在用户坐标系下的位置
    aubo_robot_namespace::Pos posOnUser;
    posOnUser.x = -0.368243;
    posOnUser.y = -1.167162;
    posOnUser.z = 0.195709;

    //轴动运动至目标位置
    robotService.robotMoveJointToTargetPosition(userCoord, posOnUser, toolEnd, true);

}
