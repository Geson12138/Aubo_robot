#include "example_2.h"
#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <vector>

using namespace std;

#define SERVER_HOST "192.168.221.13"
#define SERVER_PORT 8899

Example_2::Example_2()
{

}

void Example_2::demo()
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

    //获取机械臂当前的路点信息
    aubo_robot_namespace::wayPoint_S currentWaypoint;
    robotService.robotServiceGetCurrentWaypointInfo(currentWaypoint);

    //根据当前路点的关节角，正解得到目标路点
    aubo_robot_namespace::wayPoint_S targetWaypoint;
    ret = robotService.robotServiceRobotFk(currentWaypoint.jointpos, aubo_robot_namespace::ARM_DOF, targetWaypoint);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "--------------------正解--------------------" << std::endl;
        std::cout << "正解得到的路点位置："
                  << " x = " << targetWaypoint.cartPos.position.x
                  << " y = " << targetWaypoint.cartPos.position.y
                  << " z = " << targetWaypoint.cartPos.position.z
                  << std::endl;
        std::cout << "正解得到的目标路点的姿态（四元数）："
                  << " w = " << targetWaypoint.orientation.w
                  << " x = " << targetWaypoint.orientation.x
                  << " y = " << targetWaypoint.orientation.y
                  << " z = " << targetWaypoint.orientation.z
                  << std::endl;
        //四元数转欧拉角
        aubo_robot_namespace::Rpy rpy;
        robotService.quaternionToRPY(targetWaypoint.orientation, rpy);
        std::cout << "正解得到的目标路点的姿态（欧拉角）:"
                  << " RX = " << rpy.rx*180/M_PI
                  << " RY = " << rpy.ry*180/M_PI
                  << " RZ = " << rpy.rz*180/M_PI
                  << std::endl;
    }
    else
    {
        std::cerr << "调用正解函数失败" << std::endl;
    }

    //根据正解得到的位置姿态，来获取逆解集
    aubo_robot_namespace::Pos position = targetWaypoint.cartPos.position; //正解得到的路点的位置
    aubo_robot_namespace::Ori orientation = targetWaypoint.orientation;//正解得到的路点的姿态
    std::vector<aubo_robot_namespace::wayPoint_S> wayPointVector;
    ret = robotService.robotServiceRobotIk(position, orientation, wayPointVector);

    std::cout << std::endl;
    std::cout << "--------------------逆解集----------------------" << std::endl;
    std::cout << "逆解集的大小： " << wayPointVector.size() << std::endl;
    for(int i = 0; i < wayPointVector.size(); i++)
    {
        std::cout << "第" << i+1 << "组解：" << std::endl;
        for(int j = 0; j < 6; j++)
        {
            std::cout << "关节" << j+ 1 << ": " << wayPointVector[i].jointpos[j]*180/M_PI << std::endl;
        }
    }



    //根据当前路点，获取最优逆解
    aubo_robot_namespace::wayPoint_S wayPoint;
    robotService.robotServiceGetCurrentWaypointInfo(currentWaypoint);
    ret = robotService.robotServiceRobotIk(currentWaypoint.jointpos, position, orientation, wayPoint);
    std::cout << std::endl;
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "----------------------最优逆解-----------------------" << std::endl;
        for(int i = 0; i < 6; i++)
        {
            std::cout << "关节" << i+ 1 << ": " << wayPoint.jointpos[i]*180/M_PI << std::endl;
        }

    }
    else
    {
        std::cerr << "调用逆解函数失败" << std::endl;
    }
}
