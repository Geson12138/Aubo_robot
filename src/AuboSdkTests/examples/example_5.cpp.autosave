#include "example_5.h"
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

Example_5::Example_5()
{

}

void Example_5::toolio()
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

    //1.设置工具端电源电压类型
    aubo_robot_namespace::ToolPowerType type = aubo_robot_namespace::OUT_24V;
    ret = robotService.robotServiceSetToolPowerVoltageType(type);

    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:设置工具端电压类型成功. 当前设置的类型:"<<type<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:设置工具端电压类型失败."<<std::endl;
    }

    //2.获取工具端电源电压类型
    aubo_robot_namespace::ToolPowerType type2;
    ret = robotService.robotServiceGetToolPowerVoltageType(type2);

    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:获取电源电压类型成功.   结果为:"<<type2<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:设置工具端电压类型失败."<<std::endl;
    }


    //3.获取工具端电源电压状态
    double ToolPowerVoltageStatus;

    ret = robotService.robotServiceGetToolPowerVoltageStatus(ToolPowerVoltageStatus);

    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:获取工具端电源电压成功  电源电压:"<<ToolPowerVoltageStatus<<std::endl;
    }
    else
    {
        std::cout<<"ERROR:获取工具端电源电压失败."<<std::endl;
    }

    //4.获取工具端所有数字量IO的状态
    std::vector<aubo_robot_namespace::RobotIoDesc> statusVector;
    ret = robotService.robotServiceGetAllToolDigitalIOStatus(statusVector);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:工具端数字量IO的状态:"<<std::endl;

        for(int i=0;i<(int)statusVector.size();i++)
        {
            std::cout <<"名称:"<<statusVector[i].ioName <<" 类型:"<<statusVector[i].ioType  <<"  地址:"<<statusVector[i].ioAddr  <<"  状态:"<<statusVector[i].ioValue <<std::endl;
        }

        std::cout<<"INFO:获取工具端数字量IO的状态成功."<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:获取工具端数字量IO的状态失败."<<std::endl;
    }

    //5.设置工具端数字量IO的类型：输入或者输出
    aubo_robot_namespace::ToolDigitalIOAddr addr;
    aubo_robot_namespace::ToolIOType value;

    addr = aubo_robot_namespace::TOOL_DIGITAL_IO_1;
    value = aubo_robot_namespace::IO_OUT;

    ret = robotService.robotServiceSetToolDigitalIOType(addr, value);
    if( ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:设置工具端数字量IO的类型成功. 当前设置 addr:"<<addr<<"  类型:"<<value<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:设置工具端数字量IO的类型失败. addr:"<<addr<<std::endl;
    }

    //6.根据名称设置工具端数字量IO的状态
    std::string name = "T_DI/O_01";
    aubo_robot_namespace::IO_STATUS value2 = aubo_robot_namespace::IO_STATUS_VALID;
    robotService.robotServiceSetToolDOStatus(name, value2);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:设置工具端数字量IO的状态成功. 当前设置 name:"<<name<<"  状态:"<<value2<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:设置工具端数字量IO的状态失败. name:"<<name<<std::endl;
    }

    sleep(1);

    //7.根据地址设置工具端数字量IO的状态
    aubo_robot_namespace::ToolDigitalIOAddr addr2;
    aubo_robot_namespace::IO_STATUS value3;

    addr2 = aubo_robot_namespace::TOOL_DIGITAL_IO_2;
    value3 = aubo_robot_namespace::IO_STATUS_INVALID;

    robotService.robotServiceSetToolDOStatus(addr2, value3);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:设置工具端数字量IO的状态成功.  当前设置 addr:"<<addr2<<"  状态:"<<value3<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:设置工具端数字量IO的状态失败. addr:"<<addr2<<std::endl;
    }

    sleep(1);

    //8.获取工具端所有AI的状态
    std::vector<aubo_robot_namespace::RobotIoDesc> toolAIStatusVector;
    ret = robotService.robotServiceGetAllToolAIStatus(toolAIStatusVector);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:工具端所有AI的状态:"<<std::endl;
        for(int i=0;i<(int)toolAIStatusVector.size();i++)
        {
            std::cout <<"名称:"<<toolAIStatusVector[i].ioName <<" 类型:"<<toolAIStatusVector[i].ioType  <<"  地址:"<<toolAIStatusVector[i].ioAddr  <<"  状态:"<<toolAIStatusVector[i].ioValue <<std::endl;
        }

        std::cout<<"INFO:获取工具端所有AI的状态成功."<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:获取工具端所有AI的状态失败."<<std::endl;
    }

    //9. 设置工具端电源电压类型和所有数字量IO的类型
    ret = robotService.robotServiceSetToolPowerTypeAndDigitalIOType(aubo_robot_namespace::OUT_12V,
                                                                  aubo_robot_namespace::IO_OUT, aubo_robot_namespace::IO_OUT,
                                                                  aubo_robot_namespace::IO_OUT, aubo_robot_namespace::IO_IN);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"设置工具端IO类型SUCC."<<std::endl;
    }
    else
    {
    std::cerr<<"设置工具端IO类型Failed."<<std::endl;
    }

    //4.获取工具端所有数字量IO的状态
    std::vector<aubo_robot_namespace::RobotIoDesc> statusVector2;
    ret = robotService.robotServiceGetAllToolDigitalIOStatus(statusVector2);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:工具端数字量IO的状态:"<<std::endl;
        for(int i=0;i<(int)statusVector2.size();i++)
        {
            std::cout <<"名称:"<<statusVector2[i].ioName <<" 类型:"<<statusVector2[i].ioType  <<"  地址:"<<statusVector2[i].ioAddr  <<"  状态:"<<statusVector2[i].ioValue <<std::endl;
        }

            std::cout<<"INFO:获取工具端数字量IO的状态成功."<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:获取工具端数字量IO的状态失败."<<std::endl;
    }

    //2.获取工具端电源电压类型
    aubo_robot_namespace::ToolPowerType type3;
    ret = robotService.robotServiceGetToolPowerVoltageType(type3);

    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:获取电源电压类型成功.   结果为:"<<type3<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:设置工具端电压类型失败."<<std::endl;
    }

}

void Example_5::userio()
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

    //1.获取接口板IO配置
    ret = aubo_robot_namespace::InterfaceCallSuccCode;
    std::vector<aubo_robot_namespace::RobotIoType> ioType;
    std::vector<aubo_robot_namespace::RobotIoDesc> configVector;
    ioType.push_back(aubo_robot_namespace::RobotBoardControllerDI);
    ioType.push_back(aubo_robot_namespace::RobotBoardControllerDO);
    ioType.push_back(aubo_robot_namespace::RobotBoardControllerAI);
    ioType.push_back(aubo_robot_namespace::RobotBoardControllerAO);
    ioType.push_back(aubo_robot_namespace::RobotBoardUserDI);
    ioType.push_back(aubo_robot_namespace::RobotBoardUserDO);
    ioType.push_back(aubo_robot_namespace::RobotBoardUserAI);
    ioType.push_back(aubo_robot_namespace::RobotBoardUserAO);

    ret = robotService.robotServiceGetBoardIOConfig(ioType, configVector);

    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"获取接口版IO（包括控制柜IO和用户IO）配置成功"<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:getBoardIOConfigAPI 失败."<<std::endl;
    }

    std::cout << "ioTypeVector lenth = " << ioType.size() << std::endl;
    std::cout << "ioDescVector lenth = " << configVector.size() << std::endl;
    for(unsigned int i = 0; i < configVector.size(); i++)
    {
        std::cout << "U_DO_" << i << std::endl;
        std::cout << "ioID = " << configVector[i].ioId << " | ";
        std::cout << "ioType = " << configVector[i].ioType << " | ";
        std::cout << "ioName = " << configVector[i].ioName << " | ";
        std::cout << "ioAddr = " << configVector[i].ioAddr << " | ";
        std::cout << "ioValue = " << configVector[i].ioValue << std::endl;
    }

    //2.获取接口板IO状态
    ret = aubo_robot_namespace::InterfaceCallSuccCode;
    std::vector<aubo_robot_namespace::RobotIoType> ioType2;
    std::vector<aubo_robot_namespace::RobotIoDesc> statusVector;
    ioType2.push_back(aubo_robot_namespace::RobotBoardControllerDI);
    ioType2.push_back(aubo_robot_namespace::RobotBoardControllerDO);
    ioType2.push_back(aubo_robot_namespace::RobotBoardControllerAI);
    ioType2.push_back(aubo_robot_namespace::RobotBoardControllerAO);
    ioType2.push_back(aubo_robot_namespace::RobotBoardUserDI);
    ioType2.push_back(aubo_robot_namespace::RobotBoardUserDO);
    ioType2.push_back(aubo_robot_namespace::RobotBoardUserAI);
    ioType2.push_back(aubo_robot_namespace::RobotBoardUserAO);

    std::cout << std::endl;

    ret = robotService.robotServiceGetBoardIOStatus(ioType2, statusVector);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"获取接口版IO（包括控制柜IO和用户IO）状态成功"<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:getBoardIOStatusAPI 失败."<<std::endl;
    }

    std::cout << "ioTypeVector length = " << ioType2.size() << std::endl;
    std::cout << "ioDescVector length = " << statusVector.size() << std::endl;
    for(int i = 0; i < statusVector.size(); i++)
    {
        std::cout << "U_DO_" << i << std::endl;
        std::cout << "ioID = " << statusVector[i].ioId << " | ";
        std::cout << "ioType = " << statusVector[i].ioType << " | ";
        std::cout << "ioName = " << statusVector[i].ioName << " | ";
        std::cout << "ioAddr = " << statusVector[i].ioAddr << " | ";
        std::cout << "ioValue = " << statusVector[i].ioValue << std::endl;
    }

    //3.根据接口板IO类型和名称设置IO状态
    ret = robotService.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO, "U_DO_17", 1);
    sleep(1);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr << "根据接口板IO类型和名称设置IO状态失败!" << std::endl;
    }
    else
    {
        std::cout << "根据接口版IO类型和名称设置IO状态成功！" << std::endl;
    }

    //5.根据接口板IO类型和地址设置IO状态
    aubo_robot_namespace::RobotIoType iotype3 = aubo_robot_namespace::RobotBoardUserDO;
    int ioaddr2 = 40;
    double iovalue2 = 1;
    ret = robotService.robotServiceSetBoardIOStatus(iotype3,ioaddr2,iovalue2);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr << "根据接口板IO类型和地址设置IO状态失败!" << std::endl;
    }
    else
    {
        std::cout << "根据接口版IO类型和地址设置IO状态成功！" << std::endl;
    }
    sleep(1);
    std::cout << std::endl;


    //4.根据接口板IO类型和名称获取IO状态
    double value;
    robotService.robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO, "U_DO_02", value);
    std::cerr<<"DO_02状态:"<<value<<std::endl;

    //6.根据接口板IO类型和地址获取IO状态
    //获取 U_DI_00 的状态
    aubo_robot_namespace::RobotIoType iotype4 = aubo_robot_namespace::RobotBoardUserDI;
    int ioaddr3 = 36;
    double iovalue3;
    robotService.robotServiceGetBoardIOStatus(iotype4,ioaddr3,iovalue3);
    std::cout << "addr " << ioaddr3 << " = " << iovalue3 << std::endl;
    //获取 U_DO_00 的状态
    iotype4 = aubo_robot_namespace::RobotBoardUserDO;
    ioaddr3 = 40;
    robotService.robotServiceGetBoardIOStatus(iotype4,ioaddr3,iovalue3);
    std::cout << "addr " << ioaddr3 << " = " << iovalue3 << std::endl;
}

void Example_5::reduceMode()
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

    //退出缩减模式
    robotService.robotServiceExitRobotReduceMode();
    //robotService.robotServiceEnableRegulateSpeedMode(true);

    aubo_robot_namespace::RobotSafetyConfig speed_config;
    speed_config.robotReducedConfigJointSpeed[0] = 15 / 180.0*M_PI;
    speed_config.robotReducedConfigJointSpeed[1] = 15 / 180.0*M_PI;
    speed_config.robotReducedConfigJointSpeed[2] = 15 / 180.0*M_PI;
    speed_config.robotReducedConfigJointSpeed[3] = 15 / 180.0*M_PI;
    speed_config.robotReducedConfigJointSpeed[4] = 15 / 180.0*M_PI;
    speed_config.robotReducedConfigJointSpeed[5] = 15 / 180.0*M_PI;
    speed_config.robotReducedConfigTcpSpeed = 10;
    robotService.robotServiceSetRobotSafetyConfig(speed_config);

    //进入缩减模式
    robotService.robotServiceEnterRobotReduceMode();

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
    jointAngle[0] = 60.443151*M_PI/180;
    jointAngle[1] = 42.275463*M_PI/180;
    jointAngle[2] = -97.679737*M_PI/180;
    jointAngle[3] = -49.990510*M_PI/180;
    jointAngle[4] = -90.007372*M_PI/180;
    jointAngle[5] = 62.567046*M_PI/180;
    //关节运动
    robotService.robotServiceJointMove(jointAngle, true);

    //退出缩减模式
    robotService.robotServiceExitRobotReduceMode();
}
