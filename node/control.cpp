/*
 * sweepRobotnode.cpp
 *
 */
#include "global.h"
#include "command.h"
#include "myRobot.h"
#include <serial/serial.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

bool mainReady = false;
void callback(const std_msgs::Int32 &msg);
void callbackMain(const std_msgs::Bool &msg);

int main(int argc, char **argv) {
    MyRobot* pmyData = getMyData();
    std_msgs::UInt8MultiArray r_buffer;
    ros::init(argc, argv, "control_node");
    ros::NodeHandle n("~");
    ros::Subscriber command_key = n.subscribe("/keyboard", 10, callback);
    ros::Subscriber main_key = n.subscribe("/main_ready", 10, callbackMain);
    //ros::Publisher control_pub = n.advertise<std_msgs::Bool>("/control_ready", 5);
    ROS_INFO("control_node init.");

    int freqControl = 1000;
    int freqstatic = 5;
    static uint8_t movingLast = 0;
    // 打开传感器开关
    ros::Rate loop_rate(freqControl);

//    while(!mainReady)
//    {
//        ros::spinOnce();
//    }

    try {
        ros_ser.setPort("/dev/ttyUSB0");
        ros_ser.setBaudrate(460800);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ros_ser.setTimeout(to);
        ros_ser.open();
    }
    catch (serial::IOException &e) {
        // 初次使用需设置串口权限
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if (ros_ser.isOpen())
        ROS_INFO_STREAM("Serial Port opened");
    else
        return -1;

    int freqSerial = 400;
    uint8_t freqCount = 0;
    // 打开传感器开关
    while (ros::ok()) {
        if (ros_ser.available()) {
            std_msgs::UInt8MultiArray serial_data;
            size_t p = ros_ser.available(); //获取串口数据个数
            ros_ser.read(serial_data.data, p);
            HandleUART(serial_data.data);
        }

        if (pmyData->isAllOn && pmyData->allSensorEnable && pmyData->sendRegular) {
            ROS_INFO("all sensors on.");
            break;
        }

        if (!pmyData->isAllOn)
            TurnAllSwitch(1);
        if (!pmyData->allSensorEnable)
            AskSensorStatus();
        if (!pmyData->sendRegular)
            AskReportRegularly();
        ros::spinOnce();
        loop_rate.sleep();
    }

    //std_msgs::Bool controlReady;
    //controlReady.data = true;
    //control_pub.publish(controlReady);
    ROS_INFO("control ready!");
    uint32_t  cnt =0;
    while (ros::ok()) {
        cnt++;
        if((cnt%(freqControl/freqstatic))==0)
        {
            if ((pmyData->moving == 0 || movingLast == 0))
            {
                Move(0.f, 0.f);
            }
            movingLast = pmyData->moving;
            pmyData->moving = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void callbackMain(const std_msgs::Bool &msg) {

    mainReady = msg.data;
}

void callback(const std_msgs::Int32 &msg) {
    MyRobot* pmyData = getMyData();
    ROS_INFO(to_string(msg.data).c_str());
    switch (msg.data) {
        // esc
        case 27:
            break;
            //w
        case 119:
            ROS_INFO("wf\r\n");
            if (!BIT_0(pmyData->moving)) {
                pmyData->moving |= 0x01;
                ROS_INFO("ws\r\n");
                Move(0.3f, 0.f);
                ROS_INFO("w Move\r\n");
            }
            break;
            //    s
        case 115:
            if (!BIT_1(pmyData->moving)) {
                pmyData->moving |= 0x02;
                Move(-0.3f, 0.f);
            }
            break;
            //a
        case 97:
            if (!BIT_2(pmyData->moving)) {
                pmyData->moving |= 0x04;
                Move(0.0f, -0.5f);
            }
            break;
            //    d
        case 100:
            if (!BIT_3(pmyData->moving)) {
                pmyData->moving |= 0x08;
                Move(0.0f, 0.5f);
            }
            break;
            //    space
        case 32:
            Move(0.0f, 0.f);
            break;
            //    1
        case 49:

            break;
            //    2
        case 50:

            break;
            //    3
        case 51:

            break;
            //    4
        case 52:

            break;
            //    5
        case 53:

            break;
        default:
            break;
    }
}