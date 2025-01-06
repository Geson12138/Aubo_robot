#include "example_0.h"
#include "example_1.h"
#include "example_2.h"
#include "example_3.h"
#include "example_4.h"
#include "example_5.h"
#include "example_6.h"
#include "example_movej.h"
#include "example_followmode.h"
#include "example_movel.h"
#include "example_moverelative.h"
#include "example_moverotate.h"
#include "example_movetrack.h"
#include "example_teachmove.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "example_1_node");

    // 案例0:使用SDK构建一个最简单的机械臂的控制工程
    // Example_0::demo();

    // 案例1:回调函数的方式获取实时路点，末端速度，机械臂的事件，关节状态
    Example_1 example;
    example.demo();

    // 保持ROS节点运行
    ros::spin();

    return 0;

    //案例2：正逆解
//    Example_2::demo();

    //案例3：坐标系变换
    //案例3.1：将法兰盘中心在基坐标系下的位置和姿态转成工具在用户坐标系下的位置和姿态
//    Example_3::baseToUserCoordinate1();
    //案例3.2：将法兰盘中心在基坐标系下的位置和姿态转成法兰盘中心在用户坐标系下的位置和姿态
//    Example_3::baseToUserCoordinate2();
    //案例3.3: 将法兰盘中心在基坐标系下的位置和姿态转成工具在基坐标系下的位置和姿态
//    Example_3::baseToUserCoordinate3();
    //案例3.4：将法兰盘中心在基坐标系下的位置和姿态转成工具在基坐标系下的位置和姿态
//    Example_3::baseToBaseAdditionalTool1();
    //案例3.5：将工具末端在用户坐标系下的位置和姿态转成法兰盘中心在基坐标系下的位置和姿态
//    Example_3::userToBaseCoordinate1();
    //案例3.6：将法兰盘中心在用户坐标系下的位置和姿态转成法兰盘中心在基坐标系下的位置和姿态
//    Example_3::userToBaseCoordinate2();
    //案例3.7：将工具末端在基坐标系下的位置和姿态转成法兰盘中心在基坐标系下的位置和姿态
//    Example_3::userToBaseCoordinate3();
    //案例3.8：将工具末端在用户坐标系下的位置转成工具末端在基坐标系下的位置
//    Example_3::userCoordPointToBasePoint1();
    //案例3.9：将法兰盘中心在基坐标系下的姿态转成工具末端在基坐标系下的姿态
//    Example_3::endOrientation2ToolOrientation1();
    //案例3.10：将工具末端在基坐标系下的姿态转成法兰盘中心在基坐标系下的姿态
//    Example_3::toolOrientation2EndOrientation1();

    //案例4：设置和获取机械臂相关的参数
//    Example_4::demo();

    //案例5：IO
    //案例5.1：工具IO
//    Example_5::toolio();
    //案例5.2：用户IO
//    Example_5::userio();
    //案例5.3：安全IO之缩减模式
//    Example_5::reduceMode();

    //案例6：TCP转CAN透传模式
    //案例6.1：获取关节状态
//    Example_6::demo1();

    //案例movej：关节运动相关
    //1:关节运动
//    Example_MoveJ::movej();
    //2：关节运动到目标位置，目标位置由偏移量给出
    //法兰盘中心在基坐标系下偏移
//    Example_MoveJ::movejToTargetPositionByRelative1();
    //3：关节运动到目标位置，目标位置由偏移量给出
    //法兰盘中心在用户坐标系下偏移
//    Example_MoveJ::movejToTargetPositionByRelative2();
    //4：关节运动到目标位置，目标位置由偏移量给出
    //工具末端在工具坐标系下偏移
//    Example_MoveJ::movejToTargetPositionByRelative3();
    //5：关节运动到目标位置，目标位置由偏移量给出
    //工具末端在基坐标系下偏移
//    Example_MoveJ::movejToTargetPositionByRelative4();
    //6：关节运动到目标位置，目标位置由偏移量给出
    //工具末端在用户坐标系下偏移
//    Example_MoveJ::movejToTargetPositionByRelative5();
    //7：关节运动到目标位置，法兰盘中心在基坐标系下的位置
//    Example_MoveJ::movejToTargetPosition1();
    //8：关节运动到目标位置，法兰盘中心在用户坐标系下的位置
//    Example_MoveJ::movejToTargetPosition2();
    //9：关节运动到目标位置，工具末端在基坐标系下的位置
//    Example_MoveJ::movejToTargetPosition3();
    //10：关节运动到目标位置，工具末端在用户坐标系下的位置
//    Example_MoveJ::movejToTargetPosition4();

    //案例FollowMode：跟随模式
    //1：基于跟随模式的轴动
//    Example_FollowMode::followModeJointMove();
    //2：跟随模式之提前到位
//    Example_FollowMode::arrivalAhead();

    //案例MoveL：直线运动相关
    //1:直线运动
    // Example_MoveL::moveL();
    //2：直线运动到目标位置，目标位置由偏移量给出
    //法兰盘中心在基坐标系下偏移
//    Example_MoveL::movelToTargetPositionByRelative1();
    //3：直线运动到目标位置，目标位置由偏移量给出
    //法兰盘中心在用户坐标系下偏移
//    Example_MoveL::movelToTargetPositionByRelative2();
    //4：直线运动到目标位置，目标位置由偏移量给出
    //工具末端在工具坐标系下偏移
//    Example_MoveL::movelToTargetPositionByRelative3();
    //5：直线运动到目标位置，目标位置由偏移量给出
    //工具末端在基坐标系下偏移
//    Example_MoveL::movelToTargetPositionByRelative4();
    //6：直线运动到目标位置，目标位置由偏移量给出
    //工具末端在用户坐标系下偏移
//    Example_MoveL::movelToTargetPositionByRelative5();
    //7：直线运动到目标位置，法兰盘中心在基坐标系下的位置
//    Example_MoveL::movelToTargetPosition1();
    //8：直线运动到目标位置，法兰盘中心在用户坐标系下的位置
//    Example_MoveL::movelToTargetPosition2();
    //9：直线运动到目标位置，工具末端在基坐标系下的位置
//    Example_MoveL::movelToTargetPosition3();
    //10：直线运动到目标位置，工具末端在用户坐标系下的位置
//    Example_MoveL::movelToTargetPosition4();

    //案例MoveRelative：偏移运动
    //1：法兰盘中心在基坐标系下
//    Example_MoveRelative::demo1();
    //2：法兰盘中心在用户坐标系下
//    Example_MoveRelative::demo2();
    //3：工具末端在工具坐标系下
//    Example_MoveRelative::demo3();
    //4：工具末端在基坐标系下
//    Example_MoveRelative::demo4();
    //5：工具末端在用户坐标系下
//    Example_MoveRelative::demo5();

    //案例MoveRotate：旋转运动
    //1.法兰盘中心在基坐标系下
//    Example_MoveRotate::demo1();
    //2.法兰盘中心在用户坐标系下
//    Example_MoveRotate::demo2();
    //3.工具末端在工具坐标系下
//    Example_MoveRotate::demo3();
    //4.工具末端在基坐标系下
//    Example_MoveRotate::demo4();
    //5.工具末端在用户坐标系下
//    Example_MoveRotate::demo5();

    //案例MoveTrack：轨迹运动
    //1.圆弧
//    Example_MoveTrack::demo1();
    //2.圆
//    Example_MoveTrack::demo2();
    //3.MOVEP
//    Example_MoveTrack::demo3();

    //案例TeachMove：示教运动
    //法兰盘中心在基坐标系下
//    Example_TeachMove::demo();
}
