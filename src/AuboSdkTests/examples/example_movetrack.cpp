#include "example_movetrack.h"
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

Example_MoveTrack::Example_MoveTrack()
{

}

void Example_MoveTrack::demo1()
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

    //设置末端型运动最大加速度
    double lineMaxAcc = 0.2;
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(lineMaxAcc);

    //设置末端型运动最大速度
    double lineMaxVelc = 0.2;
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(lineMaxVelc);

    double jointAngle1[6] = {};
    jointAngle1[0] = 60.443151*M_PI/180;
    jointAngle1[1] = 42.275463*M_PI/180;
    jointAngle1[2] = -97.679737*M_PI/180;
    jointAngle1[3] = -49.990510*M_PI/180;
    jointAngle1[4] = -90.007372*M_PI/180;
    jointAngle1[5] = 62.567046*M_PI/180;

    double jointAngle2[6] = {};
    jointAngle2[0] = 83.411541*M_PI/180;
    jointAngle2[1] = 39.625360*M_PI/180;
    jointAngle2[2] = -103.796807*M_PI/180;
    jointAngle2[3] = -53.491856*M_PI/180;
    jointAngle2[4] = -90.021641*M_PI/180;
    jointAngle2[5] = 85.530279*M_PI/180;

    double jointAngle3[6] = {};
    jointAngle3[0] = 81.206455*M_PI/180;
    jointAngle3[1] = 28.381980*M_PI/180;
    jointAngle3[2] = -129.233955*M_PI/180;
    jointAngle3[3] = -67.700289*M_PI/180;
    jointAngle3[4] = -90.019516*M_PI/180;
    jointAngle3[5] = 83.325883*M_PI/180;

    //关节运动
    robotService.robotServiceJointMove(jointAngle1, true);

    //清空全局路点容器
    robotService.robotServiceClearGlobalWayPointVector();

    //添加路点
    robotService.robotServiceAddGlobalWayPoint(jointAngle1);
    //添加路点
    robotService.robotServiceAddGlobalWayPoint(jointAngle2);
    //添加路点
    robotService.robotServiceAddGlobalWayPoint(jointAngle3);

    //设置圆弧运动圈数
    robotService.robotServiceSetGlobalCircularLoopTimes(0);

    //圆弧运动
    robotService.robotServiceTrackMove(aubo_robot_namespace::ARC_CIR, true);
}

void Example_MoveTrack::demo2()
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

    //设置末端型运动最大加速度
    double lineMaxAcc = 0.2;
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(lineMaxAcc);

    //设置末端型运动最大速度
    double lineMaxVelc = 0.2;
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(lineMaxVelc);

    double jointAngle1[6] = {};
    jointAngle1[0] = 60.443151*M_PI/180;
    jointAngle1[1] = 42.275463*M_PI/180;
    jointAngle1[2] = -97.679737*M_PI/180;
    jointAngle1[3] = -49.990510*M_PI/180;
    jointAngle1[4] = -90.007372*M_PI/180;
    jointAngle1[5] = 62.567046*M_PI/180;

    double jointAngle2[6] = {};
    jointAngle2[0] = 83.411541*M_PI/180;
    jointAngle2[1] = 39.625360*M_PI/180;
    jointAngle2[2] = -103.796807*M_PI/180;
    jointAngle2[3] = -53.491856*M_PI/180;
    jointAngle2[4] = -90.021641*M_PI/180;
    jointAngle2[5] = 85.530279*M_PI/180;

    double jointAngle3[6] = {};
    jointAngle3[0] = 81.206455*M_PI/180;
    jointAngle3[1] = 28.381980*M_PI/180;
    jointAngle3[2] = -129.233955*M_PI/180;
    jointAngle3[3] = -67.700289*M_PI/180;
    jointAngle3[4] = -90.019516*M_PI/180;
    jointAngle3[5] = 83.325883*M_PI/180;

    //关节运动
    robotService.robotServiceJointMove(jointAngle1, true);

    //清空全局路点容器
    robotService.robotServiceClearGlobalWayPointVector();

    //添加路点
    robotService.robotServiceAddGlobalWayPoint(jointAngle1);
    //添加路点
    robotService.robotServiceAddGlobalWayPoint(jointAngle2);
    //添加路点
    robotService.robotServiceAddGlobalWayPoint(jointAngle3);

    //设置圆运动圈数
    robotService.robotServiceSetGlobalCircularLoopTimes(3);

    //圆运动
    robotService.robotServiceTrackMove(aubo_robot_namespace::ARC_CIR, true);
}

void Example_MoveTrack::demo3()
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

    //设置末端型运动最大加速度
    double lineMaxAcc = 0.2;
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(lineMaxAcc);

    //设置末端型运动最大速度
    double lineMaxVelc = 0.2;
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(lineMaxVelc);

    double jointAngle1[6] = {};
    jointAngle1[0] = 60.443151*M_PI/180;
    jointAngle1[1] = 42.275463*M_PI/180;
    jointAngle1[2] = -97.679737*M_PI/180;
    jointAngle1[3] = -49.990510*M_PI/180;
    jointAngle1[4] = -90.007372*M_PI/180;
    jointAngle1[5] = 62.567046*M_PI/180;

    double jointAngle2[6] = {};
    jointAngle2[0] = 83.411541*M_PI/180;
    jointAngle2[1] = 39.625360*M_PI/180;
    jointAngle2[2] = -103.796807*M_PI/180;
    jointAngle2[3] = -53.491856*M_PI/180;
    jointAngle2[4] = -90.021641*M_PI/180;
    jointAngle2[5] = 85.530279*M_PI/180;

    double jointAngle3[6] = {};
    jointAngle3[0] = 81.206455*M_PI/180;
    jointAngle3[1] = 28.381980*M_PI/180;
    jointAngle3[2] = -129.233955*M_PI/180;
    jointAngle3[3] = -67.700289*M_PI/180;
    jointAngle3[4] = -90.019516*M_PI/180;
    jointAngle3[5] = 83.325883*M_PI/180;

    //关节运动
    robotService.robotServiceJointMove(jointAngle1, true);

    //清空全局路点容器
    robotService.robotServiceClearGlobalWayPointVector();

    //添加路点
    robotService.robotServiceAddGlobalWayPoint(jointAngle1);
    //添加路点
    robotService.robotServiceAddGlobalWayPoint(jointAngle2);
    //添加路点
    robotService.robotServiceAddGlobalWayPoint(jointAngle3);

    //设置交融半径
    float blendradius = 0.05;
    std::cout << "set blendradius ret is " << robotService.robotServiceSetGlobalBlendRadius(blendradius) <<
    std::endl;

    //MoveP运动
    robotService.robotServiceTrackMove(aubo_robot_namespace::CARTESIAN_MOVEP, true);
}
