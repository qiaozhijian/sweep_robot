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
            if(!BIT_0(myData.moving))
            {
                myData.moving |= 0x01;
                Move(-0.3f,0.f);
            }
            break;
        //    s
        case 115:
            if(!BIT_1(myData.moving))
            {
                myData.moving |= 0x02;
                Move(0.3f,0.f);
            }
            break;
            //a
        case 97:
            if(!BIT_2(myData.moving))
            {
                myData.moving |= 0x04;
                Move(0.0f,-0.5f);
            }
            break;
        //    d
        case 100:
            if(!BIT_3(myData.moving))
            {
                myData.moving |= 0x08;
                Move(0.0f,0.5f);
            }
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

    int freqSerial = 100;
    int freqControl = 5;
    uint8_t freqCount = 0;
    static uint8_t movingLast = 0;
    // 打开传感器开关
    ros::Rate loop_rate(freqSerial);
    while (ros::ok())
    {
        if(ros_ser.available())
        {
            std_msgs::UInt8MultiArray  serial_data;
            size_t p=ros_ser.available(); //获取串口数据个数
            ros_ser.read(serial_data.data,p);
            HandleUART(serial_data.data);
        }

        if(myData.isAllOn && myData.allSensorEnable && myData.sendRegular)
        {
            ROS_INFO("all sensors on.");
            break;
        }

        if(!myData.isAllOn)
            TurnAllSwitch(1);
        if(!myData.allSensorEnable)
            AskSensorStatus();
        if(!myData.sendRegular)
            AskReportRegularly();
        ros::spinOnce();
        loop_rate.sleep();
    }


    while (ros::ok())
    {
        if(ros_ser.available())
        {
            std_msgs::UInt8MultiArray  serial_data;
            size_t p=ros_ser.available();
            ros_ser.read(serial_data.data,p);

            //cout<<"read: ";
            //for(int i=0;i<serial_data.data.size();i++)
            //    cout<<setbase(16)<<int(serial_data.data[i])<<" ";
            //cout<<endl;

            HandleUART(serial_data.data);

            //ROS_INFO("Time: %fs; Gyro(x,y,z): %f, %f, %f; Acc(x,y,z): %f, %f, %f; Pulse(left,right): %d, %d.",\
            myData.chassisTime/1000.f, myData.gyro_x, myData.gyro_y, myData.gyro_z, myData.accel_x, myData.accel_y, myData.accel_z, myData.pulseLeft, myData.pulseRight);
        }

        freqCount++;
        if(freqCount==freqSerial/freqControl)
        {
            freqCount=0;
            if(myData.moving == 0 || movingLast == 0)
                Move(0.f,0.f);
            movingLast = myData.moving;
            myData.moving = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}