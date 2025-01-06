#include "example_followmode.h"
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


Example_FollowMode::Example_FollowMode()
{

}

void Example_FollowMode::arrivalAhead()
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

    //机械臂运动到零位姿态
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    jointAngle[0] = 0.0/180.0*M_PI;
    jointAngle[1] = 0.0/180.0*M_PI;
    jointAngle[2] = 0.0/180.0*M_PI;
    jointAngle[3] = 0.0/180.0*M_PI;
    jointAngle[4] = 0.0/180.0*M_PI;
    jointAngle[5] = 0.0/180.0*M_PI;

    robotService.robotServiceJointMove(jointAngle, true);


    for(int i=0; i<5; i++)
    {
        if(i%2==0)
        {
            //跟随模式之提前到位　当前仅适用于关节运动
            //设置提前到位的距离模式
            robotService.robotServiceSetArrivalAheadDistanceMode(0.2);
        }
        else
        {
            robotService.robotServiceSetNoArrivalAhead();
        }

        jointAngle[0] = 20.0/180.0*M_PI;
        jointAngle[1] = 0.0/180.0*M_PI;
        jointAngle[2] = 90.0/180.0*M_PI;
        jointAngle[3] = 0.0/180.0*M_PI;
        jointAngle[4] = 90.0/180.0*M_PI;
        jointAngle[5] = 0.0/180.0*M_PI;
        robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr << "运动1失败。错误号为：" << ret << std::endl;
            break;
        }
        else
        {
            std::cerr << "运动1成功。i = " << i << std::endl;
        }

        jointAngle[0] = 50.0/180.0*M_PI;
        jointAngle[1] = 40.0/180.0*M_PI;
        jointAngle[2] = 78.0/180.0*M_PI;
        jointAngle[3] = 20.0/180.0*M_PI;
        jointAngle[4] = 66.0/180.0*M_PI;
        jointAngle[5] = 0.0/180.0*M_PI;
        robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr << "运动2失败。错误号为：" << ret << std::endl;
             break;
        }
        else
        {
            std::cerr << "运动2成功。 i = " << i << std::endl;
        }
    }
}

void Example_FollowMode::followModeJointMove()
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

    //初始位置
    double jointAngle[6] = {};
    jointAngle[0] = 173.108713*M_PI/180;
    jointAngle[1] = -12.075005*M_PI/180;
    jointAngle[2] = -83.663342*M_PI/180;
    jointAngle[3] = -15.641249*M_PI/180;
    jointAngle[4] = -89.140000*M_PI/180;
    jointAngle[5] = -28.328713*M_PI/180;

    //关节运动到初始位置
    robotService.robotServiceJointMove(jointAngle, true);

    //基于跟随模式的关节运动
    for(int i = 0; i < 1000; i++)
    {
        jointAngle[0] = jointAngle[0] + 0.0001;
        std::cout << jointAngle[0] << std::endl;
        robotService.robotServiceFollowModeJointMove(jointAngle);
        usleep(1000*5);
    }


}
