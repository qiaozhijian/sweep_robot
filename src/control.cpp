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

extern MyRobot myData;

void callback(const std_msgs::Int32::ConstPtr &msg);

int main(int argc, char **argv) {
    std_msgs::UInt8MultiArray r_buffer;
    ros::init(argc, argv, "keyboard_node");
    ros::NodeHandle n;
    ros::Subscriber command_key = n.subscribe("keyboard", 10, callback);
    ROS_INFO("keyboard_node init.");

    int freqControl = 5;
    static uint8_t movingLast = 0;
    // 打开传感器开关
    ros::Rate loop_rate(freqControl);
    while (ros::ok()) {
        if (myData.moving == 0 || movingLast == 0)
            Move(0.f, 0.f);
        movingLast = myData.moving;
        myData.moving = 0;

        ros::spinOnce();
        loop_rate.sleep();
    }
}


void callback(const std_msgs::Int32::ConstPtr &msg) {
    switch (msg->data) {
        // esc
        case 27:
            break;
            //w
        case 119:
            if (!BIT_0(myData.moving)) {
                myData.moving |= 0x01;
                Move(0.3f, 0.f);
            }
            break;
            //    s
        case 115:
            if (!BIT_1(myData.moving)) {
                myData.moving |= 0x02;
                Move(-0.3f, 0.f);
            }
            break;
            //a
        case 97:
            if (!BIT_2(myData.moving)) {
                myData.moving |= 0x04;
                Move(0.0f, -0.5f);
            }
            break;
            //    d
        case 100:
            if (!BIT_3(myData.moving)) {
                myData.moving |= 0x08;
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