#include "example_3.h"

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

Example_3::Example_3()
{

}

void Example_3::baseToUserCoordinate1()
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

    aubo_robot_namespace::Pos flangeCenterPosOnBase;//法兰盘中心在基坐标系下的位置
    flangeCenterPosOnBase.x = 0.247248;
    flangeCenterPosOnBase.y = 0.280197;
    flangeCenterPosOnBase.z = 0.322796;

    aubo_robot_namespace::Rpy flangeCenterRpyOnBase;//法兰盘中心在基坐标系下的姿态（欧拉角）
    flangeCenterRpyOnBase.rx = -101.335228*M_PI/180;
    flangeCenterRpyOnBase.ry = 8.768851*M_PI/180;
    flangeCenterRpyOnBase.rz = 93.718178*M_PI/180;

    aubo_robot_namespace::Ori flangeCenterOriOnBase;//法兰盘中心在基坐标系下的姿态（四元数）
    robotService.RPYToQuaternion(flangeCenterRpyOnBase, flangeCenterOriOnBase);

    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;//用户坐标系
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
    toolUserCoord.toolInEndPosition.x = 0;
    toolUserCoord.toolInEndPosition.y = 0;
    toolUserCoord.toolInEndPosition.z = 0.45;
    toolUserCoord.toolInEndOrientation.w = 1;
    toolUserCoord.toolInEndOrientation.x = 0;
    toolUserCoord.toolInEndOrientation.y = 0;
    toolUserCoord.toolInEndOrientation.z = 0;
    userCoord.toolDesc = toolUserCoord;

    aubo_robot_namespace::ToolInEndDesc toolInEndDesc;
    toolInEndDesc.toolInEndPosition.x = 0;
    toolInEndDesc.toolInEndPosition.y = 0;
    toolInEndDesc.toolInEndPosition.z = 0.45;
    toolInEndDesc.toolInEndOrientation.w = 1;
    toolInEndDesc.toolInEndOrientation.x = 0;
    toolInEndDesc.toolInEndOrientation.y = 0;
    toolInEndDesc.toolInEndOrientation.z = 0;

    aubo_robot_namespace::Pos toolEndPosOnUser;//工具末端在用户坐标系下的位置
    aubo_robot_namespace::Ori toolEndOriOnUser;//工具末端在用户坐标系下的姿态

    //将法兰盘中心在基坐标系下的位置姿态转化为工具末端在用户坐标系下的位置和姿态
    robotService.baseToUserCoordinate(flangeCenterPosOnBase, flangeCenterOriOnBase, userCoord, toolInEndDesc, toolEndPosOnUser, toolEndOriOnUser);

    std::cout << "工具末端在用户坐标系下的位置: ";
    std::cout << "(" << toolEndPosOnUser.x << ", " << toolEndPosOnUser.y << ", " << toolEndPosOnUser.z << ")";
    std::cout << std::endl;

    std::cout << "工具末端在用户坐标系下的姿态（四元数）: ";
    std::cout << "(" << toolEndOriOnUser.w << ", " << toolEndOriOnUser.x << ", " << toolEndOriOnUser.y << ", " << toolEndOriOnUser.z << ")";
    std::cout << std::endl;

    aubo_robot_namespace::Rpy toolEndRpyOnUser;
    robotService.quaternionToRPY(toolEndOriOnUser, toolEndRpyOnUser);
    std::cout << "工具末端在用户坐标系下的姿态（欧拉角）: ";
    std::cout << "(" << toolEndRpyOnUser.rx*180/M_PI << ", " << toolEndRpyOnUser.ry*180/M_PI << ", " << toolEndRpyOnUser.rz*180/M_PI << ")";
    std::cout << std::endl;

}

void Example_3::baseToUserCoordinate2()
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

    aubo_robot_namespace::Pos flangeCenterPosOnBase;//法兰盘中心在基坐标系下的位置
    flangeCenterPosOnBase.x = 0.247248;
    flangeCenterPosOnBase.y = 0.280197;
    flangeCenterPosOnBase.z = 0.322796;

    aubo_robot_namespace::Rpy flangeCenterRpyOnBase;//法兰盘中心在基坐标系下的姿态（欧拉角）
    flangeCenterRpyOnBase.rx = -101.335228*M_PI/180;
    flangeCenterRpyOnBase.ry = 8.768851*M_PI/180;
    flangeCenterRpyOnBase.rz = 93.718178*M_PI/180;

    aubo_robot_namespace::Ori flangeCenterOriOnBase;//法兰盘中心在基坐标系下的姿态（四元数）
    robotService.RPYToQuaternion(flangeCenterRpyOnBase, flangeCenterOriOnBase);

    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;//用户坐标系
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
    toolUserCoord.toolInEndPosition.x = 0;
    toolUserCoord.toolInEndPosition.y = 0;
    toolUserCoord.toolInEndPosition.z = 0.45;
    toolUserCoord.toolInEndOrientation.w = 1;
    toolUserCoord.toolInEndOrientation.x = 0;
    toolUserCoord.toolInEndOrientation.y = 0;
    toolUserCoord.toolInEndOrientation.z = 0;
    userCoord.toolDesc = toolUserCoord;

    aubo_robot_namespace::ToolInEndDesc toolInEndDesc;
    toolInEndDesc.toolInEndPosition.x = 0;
    toolInEndDesc.toolInEndPosition.y = 0;
    toolInEndDesc.toolInEndPosition.z = 0;
    toolInEndDesc.toolInEndOrientation.w = 1;
    toolInEndDesc.toolInEndOrientation.x = 0;
    toolInEndDesc.toolInEndOrientation.y = 0;
    toolInEndDesc.toolInEndOrientation.z = 0;

    aubo_robot_namespace::Pos flangeCenterPosOnUser;
    aubo_robot_namespace::Ori flangeCenterOriOnUser;

    //将法兰盘中心在基坐标系下的位置和姿态转成法兰盘中心在用户坐标系下的位置和姿态
    robotService.baseToUserCoordinate(flangeCenterPosOnBase, flangeCenterOriOnBase, userCoord, toolInEndDesc, flangeCenterPosOnUser, flangeCenterOriOnUser);

    std::cout << "法兰盘中心在用户坐标系下的位置: ";
    std::cout << "(" << flangeCenterPosOnUser.x << ", " << flangeCenterPosOnUser.y << ", " << flangeCenterPosOnUser.z << ")";
    std::cout << std::endl;

    std::cout << "法兰盘中心在用户坐标系下的姿态（四元数）: ";
    std::cout << "(" << flangeCenterOriOnUser.w << ", " << flangeCenterOriOnUser.x << ", " << flangeCenterOriOnUser.y << ", " << flangeCenterOriOnUser.z << ")";
    std::cout << std::endl;

    aubo_robot_namespace::Rpy flangeCenterRpyOnUser;
    robotService.quaternionToRPY(flangeCenterOriOnUser, flangeCenterRpyOnUser);
    std::cout << "法兰盘中心在用户坐标系下的姿态（欧拉角）: ";
    std::cout << "(" << flangeCenterRpyOnUser.rx*180/M_PI << ", " << flangeCenterRpyOnUser.ry*180/M_PI << ", " << flangeCenterRpyOnUser.rz*180/M_PI << ")";
    std::cout << std::endl;

}

void Example_3::baseToUserCoordinate3()
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

    aubo_robot_namespace::Pos flangeCenterPosOnBase;//法兰盘中心在基坐标系下的位置
    flangeCenterPosOnBase.x = -0.374741;
    flangeCenterPosOnBase.y = -0.534099;
    flangeCenterPosOnBase.z = 0.198738;

    aubo_robot_namespace::Rpy flangeCenterRpyOnBase;//法兰盘中心在基坐标系下的姿态（欧拉角）
    flangeCenterRpyOnBase.rx = 179.956696*M_PI/180;
    flangeCenterRpyOnBase.ry = -5.516838*M_PI/180*M_PI/180;
    flangeCenterRpyOnBase.rz = -91.337303*M_PI/180;

    aubo_robot_namespace::Ori flangeCenterOriOnBase;//法兰盘中心在基坐标系下的姿态（四元数）
    robotService.RPYToQuaternion(flangeCenterRpyOnBase, flangeCenterOriOnBase); //欧拉角转四元数

    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool baseCoord;//基坐标系
    baseCoord.coordType = aubo_robot_namespace::BaseCoordinate;

    aubo_robot_namespace::ToolInEndDesc toolInEndDesc;//工具参数
    toolInEndDesc.toolInEndPosition.x = 0;
    toolInEndDesc.toolInEndPosition.y = 0;
    toolInEndDesc.toolInEndPosition.z = 0.45;
    toolInEndDesc.toolInEndOrientation.w = 1;
    toolInEndDesc.toolInEndOrientation.x = 0;
    toolInEndDesc.toolInEndOrientation.y = 0;
    toolInEndDesc.toolInEndOrientation.z = 0;

    aubo_robot_namespace::Pos toolEndPosOnBase;//工具末端在基坐标系下的位置
    aubo_robot_namespace::Ori toolEndOriOnBase;//工具末端在基坐标系下的姿态

    //将法兰盘中心在基坐标系下的位置和姿态转成工具在基坐标系下的位置和姿态
    robotService.baseToUserCoordinate(flangeCenterPosOnBase, flangeCenterOriOnBase, baseCoord, toolInEndDesc, toolEndPosOnBase, toolEndOriOnBase);

    std::cout << "工具末端在基坐标系下的位置: ";
    std::cout << "(" << toolEndPosOnBase.x << ", " << toolEndPosOnBase.y << ", " << toolEndPosOnBase.z << ")";
    std::cout << std::endl;

    std::cout << "工具末端在基坐标系下的姿态（四元数）: ";
    std::cout << "(" << toolEndOriOnBase.w << ", " << toolEndOriOnBase.x << ", " << toolEndOriOnBase.y << ", " << toolEndOriOnBase.z << ")";
    std::cout << std::endl;

    aubo_robot_namespace::Rpy toolEndRpyOnBase;
    robotService.quaternionToRPY(toolEndOriOnBase, toolEndRpyOnBase);
    std::cout << "工具末端在基坐标系下的姿态（欧拉角）: ";
    std::cout << "(" << toolEndRpyOnBase.rx*180/M_PI << ", " << toolEndRpyOnBase.ry*180/M_PI << ", " << toolEndRpyOnBase.rz*180/M_PI << ")";
    std::cout << std::endl;
}

void Example_3::baseToBaseAdditionalTool1()
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

    aubo_robot_namespace::Pos flangeCenterPosOnBase;//法兰盘中心在基坐标系下的位置
    flangeCenterPosOnBase.x = 0.247248;
    flangeCenterPosOnBase.y = 0.280197;
    flangeCenterPosOnBase.z = 0.322796;

    aubo_robot_namespace::Rpy flangeCenterRpyOnBase;//法兰盘中心在基坐标系下的姿态（欧拉角）
    flangeCenterRpyOnBase.rx = -101.335228*M_PI/180;
    flangeCenterRpyOnBase.ry = 8.768851*M_PI/180;
    flangeCenterRpyOnBase.rz = 93.718178*M_PI/180;

    aubo_robot_namespace::Ori flangeCenterOriOnBase;//法兰盘中心在基坐标系下的姿态（四元数）
    robotService.RPYToQuaternion(flangeCenterRpyOnBase, flangeCenterOriOnBase);

    aubo_robot_namespace::ToolInEndDesc toolInEndDesc;//工具参数
    toolInEndDesc.toolInEndPosition.x = 0;
    toolInEndDesc.toolInEndPosition.y = 0;
    toolInEndDesc.toolInEndPosition.z = 0.45;
    toolInEndDesc.toolInEndOrientation.w = 1;
    toolInEndDesc.toolInEndOrientation.x = 0;
    toolInEndDesc.toolInEndOrientation.y = 0;
    toolInEndDesc.toolInEndOrientation.z = 0;

    aubo_robot_namespace::Pos toolEndPosOnBase;//工具末端在基坐标系下的位置
    aubo_robot_namespace::Ori toolEndOriOnBase;//工具末端在基坐标系下的姿态

    //将法兰盘中心在基坐标系下的位置和姿态转成工具在基坐标系下的位置和姿态
    robotService.baseToBaseAdditionalTool(flangeCenterPosOnBase, flangeCenterOriOnBase, toolInEndDesc, toolEndPosOnBase, toolEndOriOnBase);

    std::cout << "工具末端在基坐标系下的位置: ";
    std::cout << "(" << toolEndPosOnBase.x << ", " << toolEndPosOnBase.y << ", " << toolEndPosOnBase.z << ")";
    std::cout << std::endl;

    std::cout << "工具末端在基坐标系下的姿态（四元数）: ";
    std::cout << "(" << toolEndOriOnBase.w << ", " << toolEndOriOnBase.x << ", " << toolEndOriOnBase.y << ", " << toolEndOriOnBase.z << ")";
    std::cout << std::endl;

    aubo_robot_namespace::Rpy toolEndRpyOnBase;
    robotService.quaternionToRPY(toolEndOriOnBase, toolEndRpyOnBase);
    std::cout << "工具末端在基坐标系下的姿态（欧拉角）: ";
    std::cout << "(" << toolEndRpyOnBase.rx*180/M_PI << ", " << toolEndRpyOnBase.ry*180/M_PI << ", " << toolEndRpyOnBase.rz*180/M_PI << ")";
    std::cout << std::endl;

}

void Example_3::userToBaseCoordinate1()
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

    aubo_robot_namespace::Pos toolEndPosOnUser;//工具末端在用户坐标系下的位置
    toolEndPosOnUser.x = 0.108191;
    toolEndPosOnUser.y = -0.319869;
    toolEndPosOnUser.z = 0.595867;

    aubo_robot_namespace::Ori toolEndOriOnUser;//工具末端在用户坐标系下的姿态（四元数）
    aubo_robot_namespace::Rpy toolEndRpyOnUser;//工具末端在用户坐标系下的姿态（欧拉角）
    toolEndRpyOnUser.rx = -101.335*M_PI/180;
    toolEndRpyOnUser.ry = 8.76846*M_PI/180;
    toolEndRpyOnUser.rz = 93.7183*M_PI/180;
    robotService.RPYToQuaternion(toolEndRpyOnUser,toolEndOriOnUser);

    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;//用户坐标系
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
    toolUserCoord.toolInEndPosition.x = 0;
    toolUserCoord.toolInEndPosition.y = 0;
    toolUserCoord.toolInEndPosition.z = 0.45;
    toolUserCoord.toolInEndOrientation.w = 1;
    toolUserCoord.toolInEndOrientation.x = 0;
    toolUserCoord.toolInEndOrientation.y = 0;
    toolUserCoord.toolInEndOrientation.z = 0;
    userCoord.toolDesc = toolUserCoord;

    aubo_robot_namespace::ToolInEndDesc toolInEndDesc;
    toolInEndDesc.toolInEndPosition.x = 0;
    toolInEndDesc.toolInEndPosition.y = 0;
    toolInEndDesc.toolInEndPosition.z = 0.45;
    toolInEndDesc.toolInEndOrientation.w = 1;
    toolInEndDesc.toolInEndOrientation.x = 0;
    toolInEndDesc.toolInEndOrientation.y = 0;
    toolInEndDesc.toolInEndOrientation.z = 0;

    aubo_robot_namespace::Pos flangeCenterPosOnBase;//法兰盘中心在基坐标系下的位置
    aubo_robot_namespace::Ori flangeCenterOriOnBase;//法兰盘中心在基坐标系下的姿态（四元数）

    //将工具末端在用户坐标系下的位置和姿态转成法兰盘中心在基坐标系下的位置和姿态
    robotService.userToBaseCoordinate(toolEndPosOnUser, toolEndOriOnUser, userCoord, toolInEndDesc, flangeCenterPosOnBase, flangeCenterOriOnBase);

    std::cout << "法兰盘中心在基坐标系下的位置: ";
    std::cout << "(" << flangeCenterPosOnBase.x << ", " << flangeCenterPosOnBase.y << ", " << flangeCenterPosOnBase.z << ")";
    std::cout << std::endl;

    std::cout << "法兰盘中心在基坐标系下的姿态（四元数）: ";
    std::cout << "(" << flangeCenterOriOnBase.w << ", " << flangeCenterOriOnBase.x << ", " << flangeCenterOriOnBase.y << ", " << flangeCenterOriOnBase.z << ")";
    std::cout << std::endl;

    aubo_robot_namespace::Rpy flangeCenterRpyOnBase;//法兰盘中心在基坐标系下的姿态（欧拉角）
    robotService.quaternionToRPY(flangeCenterOriOnBase, flangeCenterRpyOnBase);
    std::cout << "法兰盘中心在基坐标系下的姿态（欧拉角）: ";
    std::cout << "(" << flangeCenterRpyOnBase.rx*180/M_PI << ", " << flangeCenterRpyOnBase.ry*180/M_PI << ", " << flangeCenterRpyOnBase.rz*180/M_PI << ")";
    std::cout << std::endl;

}

void Example_3::userToBaseCoordinate2()
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

    aubo_robot_namespace::Pos flangeCenterPosOnUser;//法兰盘中心在用户坐标系下的位置
    flangeCenterPosOnUser.x = 0.547609;
    flangeCenterPosOnUser.y = -0.2778;
    flangeCenterPosOnUser.z = 0.683283;

    aubo_robot_namespace::Ori flangeCenterOriOnUser;//法兰盘中心在用户坐标系下的姿态（四元数）
    aubo_robot_namespace::Rpy flangeCenterRpyOnUser;//法兰盘中心在用户坐标系下的姿态（欧拉角）
    flangeCenterRpyOnUser.rx = -101.335*M_PI/180;
    flangeCenterRpyOnUser.ry = 8.76846*M_PI/180;
    flangeCenterRpyOnUser.rz = 93.7183*M_PI/180;
    robotService.RPYToQuaternion(flangeCenterRpyOnUser,flangeCenterOriOnUser);

    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;//用户坐标系
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
    toolUserCoord.toolInEndPosition.x = 0;
    toolUserCoord.toolInEndPosition.y = 0;
    toolUserCoord.toolInEndPosition.z = 0.45;
    toolUserCoord.toolInEndOrientation.w = 1;
    toolUserCoord.toolInEndOrientation.x = 0;
    toolUserCoord.toolInEndOrientation.y = 0;
    toolUserCoord.toolInEndOrientation.z = 0;
    userCoord.toolDesc = toolUserCoord;

    aubo_robot_namespace::ToolInEndDesc toolInEndDesc;
    toolInEndDesc.toolInEndPosition.x = 0;
    toolInEndDesc.toolInEndPosition.y = 0;
    toolInEndDesc.toolInEndPosition.z = 0;
    toolInEndDesc.toolInEndOrientation.w = 1;
    toolInEndDesc.toolInEndOrientation.x = 0;
    toolInEndDesc.toolInEndOrientation.y = 0;
    toolInEndDesc.toolInEndOrientation.z = 0;

    aubo_robot_namespace::Pos flangeCenterPosOnBase;//法兰盘中心在基坐标系下的位置
    aubo_robot_namespace::Ori flangeCenterOriOnBase;//法兰盘中心在基坐标系下的姿态（四元数）

    //将法兰盘中心在用户坐标系下的位置和姿态转成法兰盘中心在基坐标系下的位置和姿态
    robotService.userToBaseCoordinate(flangeCenterPosOnUser, flangeCenterOriOnUser, userCoord, toolInEndDesc, flangeCenterPosOnBase, flangeCenterOriOnBase);

    std::cout << "法兰盘中心在基坐标系下的位置: ";
    std::cout << "(" << flangeCenterPosOnBase.x << ", " << flangeCenterPosOnBase.y << ", " << flangeCenterPosOnBase.z << ")";
    std::cout << std::endl;

    std::cout << "法兰盘中心在基坐标系下的姿态（四元数）: ";
    std::cout << "(" << flangeCenterOriOnBase.w << ", " << flangeCenterOriOnBase.x << ", " << flangeCenterOriOnBase.y << ", " << flangeCenterOriOnBase.z << ")";
    std::cout << std::endl;

    aubo_robot_namespace::Rpy flangeCenterRpyOnBase;//法兰盘中心在基坐标系下的姿态（欧拉角）
    robotService.quaternionToRPY(flangeCenterOriOnBase, flangeCenterRpyOnBase);
    std::cout << "法兰盘中心在基坐标系下的姿态（欧拉角）: ";
    std::cout << "(" << flangeCenterRpyOnBase.rx*180/M_PI << ", " << flangeCenterRpyOnBase.ry*180/M_PI << ", " << flangeCenterRpyOnBase.rz*180/M_PI << ")";
    std::cout << std::endl;

}

void Example_3::userToBaseCoordinate3()
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

    aubo_robot_namespace::Pos toolEndPosOnBase;//工具末端在基坐标系下的位置
    toolEndPosOnBase.x = -0.375099;
    toolEndPosOnBase.y = -0.534847;
    toolEndPosOnBase.z = -0.251261;

    aubo_robot_namespace::Ori toolEndOriOnBase;//工具末端在基坐标系下的姿态（四元数）
    aubo_robot_namespace::Rpy toolEndRpyOnBase;//工具末端在基坐标系下的姿态（欧拉角）
    toolEndRpyOnBase.rx = 179.957*M_PI/180;
    toolEndRpyOnBase.ry = -0.096287*M_PI/180;
    toolEndRpyOnBase.rz = -91.3373*M_PI/180;
    robotService.RPYToQuaternion(toolEndRpyOnBase, toolEndOriOnBase);

    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool baseCoord;
    baseCoord.coordType = aubo_robot_namespace::BaseCoordinate;

    aubo_robot_namespace::ToolInEndDesc toolInEndDesc;
    toolInEndDesc.toolInEndPosition.x = 0;
    toolInEndDesc.toolInEndPosition.y = 0;
    toolInEndDesc.toolInEndPosition.z = 0.45;
    toolInEndDesc.toolInEndOrientation.w = 1;
    toolInEndDesc.toolInEndOrientation.x = 0;
    toolInEndDesc.toolInEndOrientation.y = 0;
    toolInEndDesc.toolInEndOrientation.z = 0;

    aubo_robot_namespace::Pos flangeCenterPosOnBase;//法兰盘中心在基坐标系下的位置
    aubo_robot_namespace::Ori flangeCenterOriOnBase;//法兰盘中心在基坐标系下的姿态（四元数）

    //将工具末端在基坐标系下的位置和姿态转成法兰盘中心在基坐标系下的位置和姿态
    robotService.userToBaseCoordinate(toolEndPosOnBase, toolEndOriOnBase, baseCoord, toolInEndDesc, flangeCenterPosOnBase, flangeCenterOriOnBase);

    std::cout << "法兰盘中心在基坐标系下的位置: ";
    std::cout << "(" << flangeCenterPosOnBase.x << ", " << flangeCenterPosOnBase.y << ", " << flangeCenterPosOnBase.z << ")";
    std::cout << std::endl;

    std::cout << "法兰盘中心在基坐标系下的姿态（四元数）: ";
    std::cout << "(" << flangeCenterOriOnBase.w << ", " << flangeCenterOriOnBase.x << ", " << flangeCenterOriOnBase.y << ", " << flangeCenterOriOnBase.z << ")";
    std::cout << std::endl;

    aubo_robot_namespace::Rpy flangeCenterRpyOnBase;//法兰盘中心在基坐标系下的姿态（欧拉角）
    robotService.quaternionToRPY(flangeCenterOriOnBase, flangeCenterRpyOnBase);
    std::cout << "法兰盘中心在基坐标系下的姿态（欧拉角）: ";
    std::cout << "(" << flangeCenterRpyOnBase.rx*180/M_PI << ", " << flangeCenterRpyOnBase.ry*180/M_PI << ", " << flangeCenterRpyOnBase.rz*180/M_PI << ")";
    std::cout << std::endl;

}

void Example_3::userCoordPointToBasePoint1()
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

    aubo_robot_namespace::Pos toolEndPosOnUser;//工具末端在用户坐标系下的位置
    toolEndPosOnUser.x = -0.074733;
    toolEndPosOnUser.y = -1.092842;
    toolEndPosOnUser.z = 0.109217;

    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;//用户坐标系
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
    toolUserCoord.toolInEndPosition.x = 0;
    toolUserCoord.toolInEndPosition.y = 0;
    toolUserCoord.toolInEndPosition.z = 0.45;
    toolUserCoord.toolInEndOrientation.w = 1;
    toolUserCoord.toolInEndOrientation.x = 0;
    toolUserCoord.toolInEndOrientation.y = 0;
    toolUserCoord.toolInEndOrientation.z = 0;
    userCoord.toolDesc = toolUserCoord;

    aubo_robot_namespace::Pos toolEndPosOnBase;//工具末端在基坐标系下的位置
    //将工具末端在用户坐标系下的位置转成工具末端在基坐标系下的位置
    robotService.userCoordPointToBasePoint(toolEndPosOnUser, userCoord, toolEndPosOnBase);

    std::cout << "工具末端在基坐标系下的位置: ";
    std::cout << "(" << toolEndPosOnBase.x << ", " << toolEndPosOnBase.y << ", " << toolEndPosOnBase.z << ")";
    std::cout << std::endl;
}

void Example_3::endOrientation2ToolOrientation1()
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

    aubo_robot_namespace::Ori flangeCenterOriOnBase;//法兰盘中心在基坐标系下的姿态（四元数）
    aubo_robot_namespace::Rpy flangeCenterRpyOnBase;//法兰盘中心在基坐标系下的姿态（欧拉角）
    flangeCenterRpyOnBase.rx = -176.317383*M_PI/180;
    flangeCenterRpyOnBase.ry = 19.773378*M_PI/180;
    flangeCenterRpyOnBase.rz = -169.362442*M_PI/180;
    robotService.RPYToQuaternion(flangeCenterRpyOnBase, flangeCenterOriOnBase);

    aubo_robot_namespace::Ori toolOriParam;//工具姿态参数（四元数）
    aubo_robot_namespace::Rpy toolRpyParam;//工具姿态参数（欧拉角）
    toolRpyParam.rx = -162.014703*M_PI/180;
    toolRpyParam.ry = -25.569674*M_PI/180;
    toolRpyParam.rz = -36.962539*M_PI/180;
    robotService.RPYToQuaternion(toolRpyParam, toolOriParam);

    aubo_robot_namespace::Ori toolEndOriOnBase;//工具末端在基坐标系下的姿态（四元数）
    //将法兰盘中心在基坐标系下的姿态转成工具末端在基坐标系下的姿态
    robotService.endOrientation2ToolOrientation(toolOriParam, flangeCenterOriOnBase, toolEndOriOnBase);

    std::cout << "工具末端在基坐标系下的姿态（四元数）: ";
    std::cout << "(" << toolEndOriOnBase.w << ", " << toolEndOriOnBase.x << ", " << toolEndOriOnBase.y << ", " << toolEndOriOnBase.z << ")";
    std::cout << std::endl;

    aubo_robot_namespace::Rpy toolEndRpyOnBase;//工具末端在基坐标系下的姿态（欧拉角）
    robotService.quaternionToRPY(toolEndOriOnBase, toolEndRpyOnBase);
    std::cout << "工具末端在基坐标系下的姿态（欧拉角）: ";
    std::cout << "(" << toolEndRpyOnBase.rx*180/M_PI << ", " << toolEndRpyOnBase.ry*180/M_PI << ", " << toolEndRpyOnBase.rz*180/M_PI << ")";
    std::cout << std::endl;
}

void Example_3::toolOrientation2EndOrientation1()
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

    aubo_robot_namespace::Ori toolEndOriOnBase;//工具末端在基坐标系下的姿态（四元数）
    aubo_robot_namespace::Rpy toolEndRpyOnBase;//工具末端在基坐标系下的姿态（欧拉角）
    toolEndRpyOnBase.rx = 36.627361*M_PI/180;
    toolEndRpyOnBase.ry = 38.051857*M_PI/180;
    toolEndRpyOnBase.rz = -123.093803*M_PI/180;
    robotService.RPYToQuaternion(toolEndRpyOnBase, toolEndOriOnBase);

    aubo_robot_namespace::Ori toolOriParam;//工具姿态参数（四元数）
    aubo_robot_namespace::Rpy toolRpyParam;//工具姿态参数（欧拉角）
    toolRpyParam.rx = -162.014703*M_PI/180;
    toolRpyParam.ry = -25.569674*M_PI/180;
    toolRpyParam.rz = -36.962539*M_PI/180;
    robotService.RPYToQuaternion(toolRpyParam, toolOriParam);

    aubo_robot_namespace::Ori flangeCenterOriOnBase;//法兰盘中心在基坐标系下的姿态（四元数）
    //将工具末端在基坐标系下的姿态转成法兰盘中心在基坐标系下的姿态
    robotService.toolOrientation2EndOrientation(toolOriParam, toolEndOriOnBase, flangeCenterOriOnBase);
    std::cout << "法兰盘中心在基坐标系下的姿态（四元数）: ";
    std::cout << "(" << flangeCenterOriOnBase.w << ", " << flangeCenterOriOnBase.x << ", " << flangeCenterOriOnBase.y << ", " << flangeCenterOriOnBase.z << ")";
    std::cout << std::endl;

    aubo_robot_namespace::Rpy flangeCenterRpyOnBase;//法兰盘中心在基坐标系下的姿态（欧拉角）
    robotService.quaternionToRPY(flangeCenterOriOnBase, flangeCenterRpyOnBase);
    std::cout << "法兰盘中心在基坐标系下的姿态（欧拉角）: ";
    std::cout << "(" << flangeCenterRpyOnBase.rx*180/M_PI << ", " << flangeCenterRpyOnBase.ry*180/M_PI << ", " << flangeCenterRpyOnBase.rz*180/M_PI << ")";
    std::cout << std::endl;
}
