/*
 * sweepRobotnode.cpp
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
                Move(0.3f,0.f);
            }
            break;
        //    s
        case 115:
            if(!BIT_1(myData.moving))
            {
                myData.moving |= 0x02;
                Move(-0.3f,0.f);
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
    ros::init(argc, argv, "sweepRobotnode");
    ros::NodeHandle n;
    ros::Subscriber command_key = n.subscribe("keyboard", 10, callback);
    ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("robot_imu", 1000);
    ROS_INFO("sweepRobotnode init.");
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

            sensor_msgs::Imu imu_data;
            imu_data.header.stamp = ros::Time::now();
            //imu_data.header.frame_id = "map";

            //四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，，然后进行传递
            Eigen::Vector3d eulerAngle(myData.yaw,myData.pitch,myData.roll);
            Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond quaternion=yawAngle*pitchAngle*rollAngle;

            imu_data.orientation.x = quaternion.x();
            imu_data.orientation.y = quaternion.y();
            imu_data.orientation.z = quaternion.z();
            imu_data.orientation.w = quaternion.w();
            //线加速度
            imu_data.linear_acceleration.x = myData.accel_x;
            imu_data.linear_acceleration.y = myData.accel_y;
            imu_data.linear_acceleration.z = myData.accel_z;
            //角速度
            imu_data.angular_velocity.x = myData.accel_x;
            imu_data.angular_velocity.y = myData.accel_y;
            imu_data.angular_velocity.z = myData.accel_z;

            imu_data.angular_velocity_covariance[0] = myData.odometer_x;
            imu_data.angular_velocity_covariance[1] = myData.odometer_y;
            imu_data.angular_velocity_covariance[2] = myData.odometer_theta;
            imu_data.angular_velocity_covariance[3] = myData.tof;
            imu_data.angular_velocity_covariance[4] = float(myData.pulseLeft);
            imu_data.angular_velocity_covariance[5] = float(myData.pulseRight);

            IMU_pub.publish(imu_data);

            //ROS_INFO("Time: %fs; Gyro(x,y,z): %f, %f, %f; Acc(x,y,z): %f, %f, %f; Pulse(left,right): %d, %d.",\
            myData.chassisTime/1000.f, myData.gyro_x, myData.gyro_y, myData.gyro_z, myData.accel_x, myData.accel_y, myData.accel_z, myData.pulseLeft, myData.pulseRight);

            //ROS_INFO("Time: %fs; euler: %f, %f, %f; ordinate(x,y,theta): %f, %f, %f.",\
            myData.chassisTime/1000.f, myData.yaw, myData.pitch, myData.roll, myData.odometer_x, myData.odometer_y, myData.odometer_theta);
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