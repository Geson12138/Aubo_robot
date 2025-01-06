#include "example_1.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <std_msgs/Float64MultiArray.h>

#include "util.h"

#define SERVER_HOST "192.168.26.103"
#define SERVER_PORT 8899

ros::Publisher Example_1::joint_pub;

Example_1::Example_1()
{
    ros::NodeHandle nh;
    joint_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_angles", 10);
}

void Example_1::RealTimeWaypointCallback(const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg)
{
    (void)arg;
    aubo_robot_namespace::wayPoint_S waypoint = *wayPointPtr;
    std::cout << "实时路点信息:";
    std_msgs::Float64MultiArray joint_msg;
    for (int i = 0; i < sizeof(waypoint.jointpos) / sizeof(waypoint.jointpos[0]); ++i)
    {
        std::cout << " " << waypoint.jointpos[i];
        joint_msg.data.push_back(waypoint.jointpos[i]);
    }
    std::cout << std::endl;
    joint_pub.publish(joint_msg);
}

void Example_1::RealTimeEndSpeedCallback(double speed, void *arg)
{
    (void)arg;
    std::cout << "实时末端速度:" << speed << std::endl;
}

void Example_1::RealTimeEventInfoCallback(const aubo_robot_namespace::RobotEventInfo *pEventInfo, void *arg)
{
    (void)arg;
    Util::printEventInfo(*pEventInfo);
}

void Example_1::RealTimeJointStatusCallback(const aubo_robot_namespace::JointStatus *jointStatusPtr, int size, void *arg)
{
    (void)arg;
    Util::printJointStatus(jointStatusPtr, size);
}

void Example_1::demo()
{
    ServiceInterface robotService;


    int ret = aubo_robot_namespace::InterfaceCallSuccCode;
    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr << "登录成功." << std::endl;
    }
    else
    {
        std::cerr << "登录成功." << std::endl;
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

    //获取实时路点信息
    robotService.robotServiceRegisterRealTimeRoadPointCallback(Example_1::RealTimeWaypointCallback, NULL);

    //获取实时末端速度信息
    // robotService.robotServiceRegisterRealTimeEndSpeedCallback(Example_1::RealTimeEndSpeedCallback, NULL);

    //获取实时事件信息
    // robotService.robotServiceRegisterRobotEventInfoCallback(Example_1::RealTimeEventInfoCallback, NULL);

    //获取实时关节状态信息
    // robotService.robotServiceRegisterRealTimeJointStatusCallback(Example_1::RealTimeJointStatusCallback, NULL);

}

