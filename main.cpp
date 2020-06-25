/*
 * myserialnode.cpp
 *
 */
#include "global.h"
#include "command.h"


serial::Serial ros_ser;
extern SerialData myData;
void callback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO_STREAM("Write to serial port:" << msg->data);
    ros_ser.write(msg->data);
}


int main(int argc, char **argv) {
    std_msgs::UInt8MultiArray r_buffer;
    ros::init(argc, argv, "myserialnode");
    ros::NodeHandle n;
    ros::Subscriber command_sub = n.subscribe("keyboard", 1000, callback);

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
    ros::Rate loop_rate(100);
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



    //ros::Rate loop_rate(100);
    //while (ros::ok()) {
    //    ros_ser.write(s_buffer, 19);
    //    ROS_INFO("serial sends: 0x%x 0x%x 0X%x", s_buffer[4], s_buffer[18], test_crc);
    //    ros::spinOnce();
    //    loop_rate.sleep();
    //}
}