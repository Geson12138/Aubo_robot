#include "example_6.h"
#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include "util.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>

#define SERVER_HOST "192.168.221.13"
#define SERVER_PORT 8899

Example_6::Example_6()
{

}

void Example_6::demo1()
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

    //进入TCP转CAN透传模式
    ret = robotService.robotServiceEnterTcp2CanbusMode();
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "进入TCP转CAN透传模式成功。" << std::endl;
    }
    else
    {
        std::cerr << "进入TCP转CAN透传模式失败。错误号为" << ret << std::endl;
    }

    aubo_robot_namespace::JointStatus jointStatus[6];
    //获取机械臂关节状态
    ret = robotService.robotServiceGetRobotJointStatus(jointStatus, 6);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "获取关节状态成功。" << std::endl;
        //打印关节状态信息
        Util::printJointStatus(jointStatus, 6);
    }
    else
    {
        std::cerr << "获取关节状态失败。错误号为" << ret << std::endl;
    }

    //退出TCP转CAN透传模式
    robotService.robotServiceLeaveTcp2CanbusMode();

}
