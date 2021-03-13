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

bool mainReady = true;
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

    while(!mainReady)
    {
        ROS_ERROR("in while(!mainReady)");
        ros::spinOnce();
    }

    //return 0;

    //todo: 串口初始化
    try {
        ros_ser.setPort("/dev/ttyUSB0");
        ros_ser.setBaudrate(115200);//原460800，现根据扫地机器人说明书调
        serial::Timeout to = serial::Timeout::simpleTimeout(200);//原1000，现根据扫地机器人说明书调
        ros_ser.setTimeout(to);
        ros_ser.open();
    }
    catch (serial::IOException &e) {
        // 初次使用需设置串口权限
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if (ros_ser.isOpen()){
        //todo Check which node opens serial port
        ROS_INFO_STREAM("Serial Port opened in control");
    }
    else
        return -1;

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
    ROS_ERROR("In callbackMain");
    mainReady = msg.data;
//    if(mainReady)
//        ROS_ERROR("main ready get");
//    else
//        ROS_ERROR("main ready not get");
}

void callback(const std_msgs::Int32 &msg) {
    ROS_ERROR("In callback");
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
                ROS_INFO("w\r\n");
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