#include "example_moverotate.h"
#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>

#define SERVER_HOST "192.168.221.13"
#define SERVER_PORT 8899


Example_MoveRotate::Example_MoveRotate()
{

}

void Example_MoveRotate::demo1()
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

    //关节角
    double jointAngle[6] = {};
    jointAngle[0] = 173.108713*M_PI/180;
    jointAngle[1] = -12.075005*M_PI/180;
    jointAngle[2] = -83.663342*M_PI/180;
    jointAngle[3] = -15.641249*M_PI/180;
    jointAngle[4] = -89.140000*M_PI/180;
    jointAngle[5] = -28.328713*M_PI/180;

    //关节运动到起始位置
    robotService.robotServiceJointMove(jointAngle, true);

    //设置基坐标系
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool baseCoord;
    baseCoord.coordType = aubo_robot_namespace::BaseCoordinate;

    //旋转轴和旋转角度
    double rotateAxis[3] = {1, 0, 0};
    double rotateAngle = 10.0*M_PI/180;

    //旋转运动：法兰盘中心在基坐标系下沿X+方向旋转10度，当前位置保持不变
    robotService.robotServiceRotateMove(baseCoord, rotateAxis, rotateAngle, true);

}

void Example_MoveRotate::demo2()
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

    //关节角
    double jointAngle[6] = {};
    jointAngle[0] = 173.108713*M_PI/180;
    jointAngle[1] = -12.075005*M_PI/180;
    jointAngle[2] = -83.663342*M_PI/180;
    jointAngle[3] = -15.641249*M_PI/180;
    jointAngle[4] = -89.140000*M_PI/180;
    jointAngle[5] = -28.328713*M_PI/180;

    //关节运动到起始位置
    robotService.robotServiceJointMove(jointAngle, true);

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

    //旋转轴和旋转角度
    double rotateAxis[3] = {1, 0, 0};
    double rotateAngle = 10.0*M_PI/180;

    //旋转运动：法兰盘中心在用户坐标系下沿X+方向旋转10度，当前位置保持不变
    robotService.robotServiceRotateMove(userCoord, rotateAxis, rotateAngle, true);
}

void Example_MoveRotate::demo3()
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

    //旋转轴和旋转角度
    double rotateAxis[3] = {1, 0, 0};
    double rotateAngle = 10.0*M_PI/180;

    //旋转运动：工具末端在工具坐标系下沿X+方向旋转10度，当前位置保持不变
    robotService.robotServiceRotateMove(endCoord, rotateAxis, rotateAngle, true);

}

void Example_MoveRotate::demo4()
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

    //旋转轴和旋转角度
    double rotateAxis[3] = {1, 0, 0};
    double rotateAngle = 10.0*M_PI/180;

    //旋转运动：工具末端在基坐标系下沿X+方向旋转10度，当前位置保持不变
    robotService.robotServiceRotateMove(baseCoord, rotateAxis, rotateAngle, true);

}

void Example_MoveRotate::demo5()
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

    //旋转轴和旋转角度
    double rotateAxis[3] = {1, 0, 0};
    double rotateAngle = 10.0*M_PI/180;

    //旋转运动：工具末端在用户坐标系下沿X+方向旋转10度，当前位置保持不变
    robotService.robotServiceRotateMove(userCoord, rotateAxis, rotateAngle, true);

}
