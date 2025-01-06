#ifndef EXAMPLE_1_H
#define EXAMPLE_1_H
#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>


class Example_1
{
public:
    Example_1();

public:
    static void RealTimeWaypointCallback (const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg); //用于获取实时路点回调函数

    static void RealTimeEndSpeedCallback (double speed, void *arg);  //获取实时末端速度回调函数

    static void RealTimeEventInfoCallback(const aubo_robot_namespace::RobotEventInfo *pEventInfo, void *arg); //获取实时机械臂事件回调函数

    static void RealTimeJointStatusCallback(const aubo_robot_namespace::JointStatus *jointStatusPtr, int size, void *arg); //获取实时机械臂关节状态

    /**
     * @brief demo
     *
     * 回调函数的方式获取实时路点，末端速度，机械臂的事件
     */
    static void demo();

private:

    static ros::Publisher joint_pub;


};

#endif // EXAMPLE_1_H
