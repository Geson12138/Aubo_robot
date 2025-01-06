#include "example_teachmove.h"
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

Example_TeachMove::Example_TeachMove()
{

}

void Example_TeachMove::demo()
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
    jointMaxVelc.jointPara[0] = 15*M_PI/180;
    jointMaxVelc.jointPara[1] = 15*M_PI/180;
    jointMaxVelc.jointPara[2] = 15*M_PI/180;
    jointMaxVelc.jointPara[3] = 15*M_PI/180;
    jointMaxVelc.jointPara[4] = 15*M_PI/180;
    jointMaxVelc.jointPara[5] = 15*M_PI/180;
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    //设置示教坐标系为基坐标系
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool baseCoord;
    robotService.robotServiceSetTeachCoordinateSystem(baseCoord);

    //设置示教模式——关节示教
    aubo_robot_namespace::teach_mode teachmode1;
    teachmode1 = aubo_robot_namespace::JOINT1;

    //开始示教
    robotService.robotServiceTeachStart(teachmode1, true);
    sleep(2);

    //结束示教
    robotService.robotServiceTeachStop();

    //设置示教模式——位置示教
    aubo_robot_namespace::teach_mode teachmode2;
    teachmode2 = aubo_robot_namespace::MOV_X;

    //开始示教
    robotService.robotServiceTeachStart(teachmode2, true);
    sleep(5);
    //结束示教
    robotService.robotServiceTeachStop();

    //设置示教模式——姿态示教
    aubo_robot_namespace::teach_mode teachmode3;
    teachmode3 = aubo_robot_namespace::ROT_Z;

    //开始示教
    robotService.robotServiceTeachStart(teachmode3, true);
    sleep(5);
    //结束示教
    robotService.robotServiceTeachStop();
}
