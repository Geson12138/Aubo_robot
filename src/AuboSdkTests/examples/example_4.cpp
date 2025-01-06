#include "example_4.h"
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

Example_4::Example_4()
{

}

void Example_4::demo()
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

    //1.获取机械臂关节状态
    aubo_robot_namespace::JointStatus jointStatus[6];
    ret = robotService.robotServiceGetRobotJointStatus(jointStatus, 6);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "获取关节状态成功." << std::endl;

        Util::printJointStatus(jointStatus, 6);

    }
    else
    {
        std::cerr << "获取关节状态失败." << std::endl;
    }

    //2.获取真实臂是否存在
    bool IsRealRobotExist = false;
    ret = robotService.robotServiceGetIsRealRobotExist(IsRealRobotExist);

    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout << "真实机械臂是否存在：" << IsRealRobotExist << std::endl;
    }
    else
    {
        std::cerr << "ERROR:获取机械臂真实臂是否存在失败." << std::endl;
    }

    //3.机械臂诊断信息
    aubo_robot_namespace::RobotDiagnosis robotDiagnosisInfo;
    ret = robotService.robotServiceGetRobotDiagnosisInfo(robotDiagnosisInfo);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        Util::printRobotDiagnosis(robotDiagnosisInfo);
    }
    else
    {
        std::cerr << "ERROR:获取机械臂诊断信息失败." << std::endl;
    }

    //4.获取连接状态
    bool connectStatus;
    robotService.robotServiceGetConnectStatus(connectStatus);
    std::cout << "机械臂的连接状态： " << connectStatus << std::endl;

    //5.设置机械臂当前工作模式
    aubo_robot_namespace::RobotWorkMode workmode = aubo_robot_namespace::RobotModeReal;
    ret = robotService.robotServiceSetRobotWorkMode(workmode);
    std::cout << "设置机械臂当前工作模式: " << workmode << std::endl;

    //6.获取机械臂当前工作模式
    aubo_robot_namespace::RobotWorkMode workmode2;
    robotService.robotServiceGetRobotWorkMode(workmode2);
    std::cout << "机械臂当前工作模式: " << workmode2 << std::endl;

    //7.获取重力分量
    aubo_robot_namespace::RobotGravityComponent gravity;
    robotService.robotServiceGetRobotGravityComponent(gravity);
    std::cout << "**** 重力分量 ****" << std::endl;
    std::cout << "x = " << gravity.x << std::endl;
    std::cout << "y = " << gravity.y << std::endl;
    std::cout << "z = " << gravity.z << std::endl;

    //8.设置机械臂碰撞等级
    int collisionClass = 5;
    ret = robotService.robotServiceSetRobotCollisionClass(collisionClass);
    std::cout << "设置机械臂碰撞等级: " << collisionClass << std::endl;

    //9.获取碰撞等级
    int collisiongrade;
    robotService.robotServiceGetRobotCollisionCurrentService(collisiongrade);
    std::cout << "获取碰撞等级: " << collisiongrade << std::endl;

    //10.获取设备信息
    aubo_robot_namespace::RobotDevInfo robotdevinfo;
    robotService.robotServiceGetRobotDevInfoService(robotdevinfo);
    std::cout << "**** 机械臂设备信息 ****" << std::endl;
    std::cout << "设备版本号 revision : " << robotdevinfo.revision << std::endl;
    std::cout << "从设备版本号 slave version : " << robotdevinfo.slave_version << std::endl;
    std::cout << "IO扩展版本号 extern IO version : " << robotdevinfo.extio_version << std::endl;
    std::cout << "厂家ID manu_id : " << robotdevinfo.manu_id << std::endl;
    std::cout << "机械臂类型 joint_type : " << robotdevinfo.joint_type << std::endl;
    for(int i = 0; i < 8; i++)
    {
        if(i == 6)
        {
          std::cout <<"Tool" << " 硬件版本 hardware version is : " << robotdevinfo.joint_ver[i].hw_version << std::endl;
          std::cout <<"Tool" << " 软件版本 software version is : " << robotdevinfo.joint_ver[i].sw_version << std::endl;
        } else if(i == 7)
        {
            std::cout <<"Base" << " 硬件版本 hardware version is : " << robotdevinfo.joint_ver[i].hw_version << std::endl;
            std::cout <<"Base" << " 软件版本 software version is : " << robotdevinfo.joint_ver[i].sw_version << std::endl;
        } else
        {
            std::cout <<"关节 joint[" << i+1 << "] 硬件版本 hardware version is : " << robotdevinfo.joint_ver[i].hw_version << std::endl;
            std::cout <<"关节 joint[" << i+1 << "] 软件版本 software version is : " << robotdevinfo.joint_ver[i].sw_version << std::endl;
        }
    }
    for(int i = 0; i < 8; i++)
    {
        if(i == 0)
        {
            std::cout <<"Interface Board ID is : " << robotdevinfo.jointProductID[i].productID << std::endl;
        } else if(i == 7)
        {
            std::cout <<"Tool ID is : " << robotdevinfo.jointProductID[i].productID << std::endl;
        } else
        {
        std::cout <<"关节 joint[" << i << "] ID is : " << robotdevinfo.jointProductID[i].productID << std::endl;
        }
    }


    //11.获取机械臂当前运行状态
    aubo_robot_namespace::RobotState robotstate;
    robotService.robotServiceGetRobotCurrentState(robotstate);
    std::cout << "机械臂当前运行状态: " << robotstate << std::endl;

    //12.获取 MAC 通信状态
    bool macconnectstate;
    robotService.robotServiceGetMacCommunicationStatus(macconnectstate);
    std::cout << "MAC 通信状态: " << macconnectstate << std::endl;

    //13.获取６关节旋转 360 使能标志
    bool j6_360_flag;
    robotService.robotServiceGetJoint6Rotate360EnableFlag(j6_360_flag);
    std::cout << "joint 6 360 flag : " << j6_360_flag << std::endl;

    //14.获取机械臂当前关节角信息
    aubo_robot_namespace::JointParam jointangle;
    robotService.robotServiceGetJointAngleInfo(jointangle);
    //关节信息
    std::cout<<"关节角: "<<std::endl;
    for(int i=0;i<aubo_robot_namespace::ARM_DOF;i++)
    {
        std::cout << "关节" << i+1 << ": "<< jointangle.jointPos[i] << " ~ " <<
        jointangle.jointPos[i]*180.0/M_PI << std::endl;
    }

    //15.获取当前路点信息
    aubo_robot_namespace::wayPoint_S wayPoint;
    robotService.robotServiceGetCurrentWaypointInfo(wayPoint);
    std::cout<<std::endl<<"-------------当前路点信息---------------"<<std::endl;
    //位置信息
    std::cout<<"位置 Pos: ";
    std::cout<<"x:"<<wayPoint.cartPos.position.x<<" ";
    std::cout<<"y:"<<wayPoint.cartPos.position.y<<" ";
    std::cout<<"z:"<<wayPoint.cartPos.position.z<<std::endl;
    //姿态信息
    std::cout<<"姿态 Ori: ";
    std::cout<<"w:"<<wayPoint.orientation.w<<" ";
    std::cout<<"x:"<<wayPoint.orientation.x<<" ";
    std::cout<<"y:"<<wayPoint.orientation.y<<" ";
    std::cout<<"z:"<<wayPoint.orientation.z<<std::endl;
    aubo_robot_namespace::Rpy tempRpy;
    robotService.quaternionToRPY(wayPoint.orientation,tempRpy);
    std::cout<<"欧拉角 Rpy: ";
    std::cout<<"RX:"<<tempRpy.rx*180.0/M_PI<<" RY:"<<tempRpy.ry*180.0/M_PI
        <<" RZ:"<<tempRpy.rz*180.0/M_PI<<std::endl;
    //关节信息
    std::cout<<"关节位置: "<<std::endl;
    for(int i=0;i<aubo_robot_namespace::ARM_DOF;i++)
    {
        std::cout<<"关节"<<i+1<<": "<<wayPoint.jointpos[i]<<" ~ "<<wayPoint.jointpos[i]*180.0/M_PI<<std::endl;
    }

    //16.是否在联机模式
    bool isonlinemode;
    robotService.robotServiceIsOnlineMode(isonlinemode);
    std::cout << "机械臂是否在联机模式 : " << isonlinemode << std::endl;

    //17.是否在联机主模式
    bool isonlinemastermode;
    robotService.robotServiceIsOnlineMasterMode(isonlinemastermode);
    std::cout << "机械臂是否在联机主模式: " << isonlinemastermode << std::endl;

    //18.获取机械臂安全配置
    aubo_robot_namespace::RobotSafetyConfig safeconfig;
    robotService.robotServiceGetRobotSafetyConfig(safeconfig);
    std::cout << "缩减模式的关节速度限制：" << std::endl;
    for(int i = 0; i < 6 ;i++)
    {
        std::cout << "关节[" << i+1 << "] 速度 = " << safeconfig.robotReducedConfigJointSpeed[i] << std::endl;
    }
    std::cout << "缩减模式的TCP速度限制 = " << safeconfig.robotReducedConfigTcpSpeed << std::endl;
    std::cout << "缩减模式的TCP力 = " << safeconfig.robotReducedConfigTcpForce << std::endl;
    std::cout << "缩减模式的动量 = " << safeconfig.robotReducedConfigMomentum << std::endl;
    std::cout << "缩减模式的功率 = " << safeconfig.robotReducedConfigPower << std::endl;
}
