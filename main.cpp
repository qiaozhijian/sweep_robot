/*
 * myserialnode.cpp
 *
 */
#include "global.h"
#include "command.h"


serial::Serial ros_ser;
extern SerialData myData;
void callback(const std_msgs::Int32 ::ConstPtr &msg) {
    switch (msg->data)
    {
        // esc
        case 27:
            break;
        //w
        case 119:
            myData.moving = true;
            Move(0.3f,0.f);
            break;
        //    s
        case 115:
            myData.moving = true;
            Move(0.3f,0.f);
            break;
            //a
        case 97:
            myData.moving = true;
            Move(0.0f,-0.5f);
            break;
        //    d
        case 100:
            myData.moving = true;
            Move(0.0f,0.5f);
            break;
        //    space
        case 32:
            Move(0.0f,0.f);
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


int main(int argc, char **argv) {
    std_msgs::UInt8MultiArray r_buffer;
    ros::init(argc, argv, "myserialnode");
    ros::NodeHandle n;
    ros::Subscriber command_key = n.subscribe("keyboard", 10, callback);
    ROS_INFO("myserialnode init.");
    //return 0;
    try {
        ros_ser.setPort("/dev/ttyUSB0");
        ros_ser.setBaudrate(115200);
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


    // 打开传感器开关
    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        TurnAllSwitch(1);
        AskSensorStatus();
        if(ros_ser.available())
        {
            std_msgs::UInt8MultiArray  serial_data;
            //获取串口数据个数
            size_t p=ros_ser.available();
            ros_ser.read(serial_data.data,p);
            GetSensorStatus(serial_data.data);
            if(myData.isAllOn)
            {
                ROS_INFO("all sensors on.");
                break;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    static bool movingLast = false;
    while (ros::ok())
    {
        if(myData.moving == false || movingLast == false)
            Move(0.f,0.f);
        movingLast = myData.moving;
        myData.moving = false;
        ros::spinOnce();
        loop_rate.sleep();
    }


    //ros::Rate loop_rate(100);
    //while (ros::ok()) {
    //    ros_ser.write(s_buffer, 19);
    //    ROS_INFO("serial sends: 0x%x 0x%x 0X%x", s_buffer[4], s_buffer[18], test_crc);
    //    ros::spinOnce();
    //    loop_rate.sleep();
    //}
}