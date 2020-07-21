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
#include "utility.h"

extern MyRobot pmyData;

int main(int argc, char **argv) {
    std_msgs::UInt8MultiArray r_buffer;
    ros::init(argc, argv, "/sweepRobotnode");
    //后面的RECORD_IMU变量的主名字
    ros::NodeHandle n("sweepRobotnode");
    ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("/imu0", 1000);
    string isIMUrecord = "0";
    //注意添加/，表示ros空间里的变量
    n.param<std::string>("RECORD_IMU", isIMUrecord, "0");
    ROS_INFO("isIMUrecord %s",isIMUrecord.c_str());
    ROS_INFO("sweepRobotnode init.");
    //return 0;
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
    int freqControl = 5;
    uint8_t freqCount = 0;
    static uint8_t movingLast = 0;
    // 打开传感器开关
    ros::Rate loop_rate(freqSerial);
    while (ros::ok()) {
        if (ros_ser.available()) {
            std_msgs::UInt8MultiArray serial_data;
            size_t p = ros_ser.available(); //获取串口数据个数
            ros_ser.read(serial_data.data, p);
            HandleUART(serial_data.data);
        }

        if (pmyData.isAllOn && pmyData.allSensorEnable && pmyData.sendRegular) {
            ROS_INFO("all sensors on.");
            break;
        }

        if (!pmyData.isAllOn)
            TurnAllSwitch(1);
        if (!pmyData.allSensorEnable)
            AskSensorStatus();
        if (!pmyData.sendRegular)
            AskReportRegularly();
        ros::spinOnce();
        loop_rate.sleep();
    }

    while(!pmyData.cameraReady);
    ROS_INFO("camera ready!");

    uint32 timeLast = 0.0;
    uint32 reportLast = 0;
    double yaw_ = 0.0;
    uint32 reportCount = 0;
    while (ros::ok()) {
        if (ros_ser.available()) {
            std_msgs::UInt8MultiArray serial_data;
            size_t p = ros_ser.available();
            ros_ser.read(serial_data.data, p);

            //cout<<"read: ";
            //for(int i=0;i<serial_data.data.size();i++)
            //    cout<<setbase(16)<<int(serial_data.data[i])<<" ";
            //cout<<endl;

            HandleUART(serial_data.data);

            if (pmyData.imuUpdate) {
                pmyData.imuReady= true;
                sensor_msgs::Imu imu_data;
                imu_data.header.stamp = ros::Time::now();
                //imu_data.header.frame_id = "map";

                //四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，，然后进行传递
                //Eigen::Vector3d eulerAngle(pmyData.yaw, pmyData.pitch, pmyData.roll);
                //Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
                //Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
                //Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
                //Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
                //tf::Quaternion orientation;
                //orientation.setEulerZYX(pmyData.yaw,pmyData.pitch,pmyData.roll);


                //imu_data.orientation.x = quaternion.x();
                //imu_data.orientation.y = quaternion.y();
                //imu_data.orientation.z = quaternion.z();
                //imu_data.orientation.w = quaternion.w();
                //线加速度
                imu_data.linear_acceleration.x = pmyData.accel_x;
                imu_data.linear_acceleration.y = pmyData.accel_y;
                imu_data.linear_acceleration.z = pmyData.accel_z;
                //角速度
                imu_data.angular_velocity.x = pmyData.gyro_x;
                imu_data.angular_velocity.y = pmyData.gyro_y;
                imu_data.angular_velocity.z = pmyData.gyro_z;

                if (isIMUrecord.compare("1")!=0) {
                    //角速度使用去过零漂的
#ifdef ROMOVE_BIAS
                    //imu_data.angular_velocity.x = pmyData.w_x_self;
                    //imu_data.angular_velocity.y = pmyData.w_y_self;
                    //imu_data.angular_velocity.z = pmyData.w_z_self;
#endif
                    imu_data.angular_velocity_covariance[0] = pmyData.odometer_x;
                    imu_data.angular_velocity_covariance[1] = pmyData.odometer_y;
                    imu_data.angular_velocity_covariance[2] = pmyData.odometer_theta;
                    imu_data.angular_velocity_covariance[3] = pmyData.tof;
                    imu_data.angular_velocity_covariance[4] = float(pmyData.pulseLeft);
                    imu_data.angular_velocity_covariance[5] = float(pmyData.pulseRight);
                    imu_data.angular_velocity_covariance[6] = pmyData.roll;
                    imu_data.angular_velocity_covariance[7] = pmyData.pitch;
                    imu_data.angular_velocity_covariance[8] = pmyData.yaw;
                }

                IMU_pub.publish(imu_data);
                pmyData.SaveRobotData(pmyData.dir + "/robot.txt",imu_data.header.stamp.toSec());
                ROS_INFO("delta time: %d",pmyData.chassisTime);
                //ROS_INFO("delta time: %d,Gyro(x,y,z): %f, %f, %f; Acc(x,y,z): %f, %f, %f; roll %f, pitch %f. yaw %f.",
                //         pmyData.chassisTime - reportLast, pmyData.gyro_x, pmyData.gyro_y, pmyData.gyro_z, pmyData.accel_x,
                //         pmyData.accel_y, pmyData.accel_z, pmyData.roll, pmyData.pitch, pmyData.yaw);
                //ROS_INFO("delta time: %d,Gyro(x,y,z): %f, %f, %f; Acc(x,y,z): %f, %f, %f; yaw %f, theta %f. pulse %d %d.", c, pmyData.gyro_x, pmyData.gyro_y, pmyData.gyro_z, pmyData.accel_x, pmyData.accel_y, pmyData.accel_z, pmyData.yaw ,pmyData.odometer_theta / 3.1415926 * 180.0,pmyData.pulseLeft,pmyData.pulseRight);
            }
            pmyData.imuUpdate = false;

            //if((reportCount++)%1000==0)
            reportLast = pmyData.chassisTime;
            //ROS_INFO("Time: %fs; Gyro(x,y,z): %f, %f, %f; Acc(x,y,z): %f, %f, %f; Pulse(left,right): %d, %d.",\
            pmyData.chassisTime/1000.f, pmyData.gyro_x, pmyData.gyro_y, pmyData.gyro_z, pmyData.accel_x, pmyData.accel_y, pmyData.accel_z, pmyData.pulseLeft, pmyData.pulseRight);
            uint32 timeMs = pmyData.chassisTime;
            if (timeLast != 0.0) {
                yaw_ = yaw_ + (timeMs - timeLast) * 0.001 * pmyData.w_z_self;
                //ROS_INFO("yaw %f; odometer_theta: %f; euler_yaw: %f",yaw_/3.1415926*180.0, pmyData.odometer_theta/3.1415926*180.0, pmyData.yaw);
            }
            timeLast = timeMs;
            //ROS_INFO("Time: %fs; euler: %f, %f, %f; ordinate(x,y,theta): %f, %f, %f.",\
            pmyData.chassisTime/1000.f, pmyData.yaw, pmyData.pitch, pmyData.roll, pmyData.odometer_x, pmyData.odometer_y, pmyData.odometer_theta);
        }

        //loop_rate.sleep();
    }
}
