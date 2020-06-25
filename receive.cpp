//
// Created by qzj on 2020/6/24.
//

#include "receive.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "crc.h"

serial::Serial ros_ser2;

void callback(const std_msgs::String::ConstPtr& msg)
{
    //ROS_INFO_STREAM("Writing to serial port" <<msg->data);
    //ser.write(msg->data);   //发送串口数据
    cout<<"come into callback"<<endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("listenTopic", 1000, callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("listenTopic", 1000);
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        if(ros_ser2.available())
        {
            ROS_INFO_STREAM("Reading from serial port\n");
            std_msgs::String result;
            result.data = ros_ser2.read(ros_ser2.available());
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}