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
#include "geometry_msgs/TransformStamped.h"


string vo_dir = "";
void callbackVO(const std_msgs::String &msg) {

    vo_dir = msg.data.c_str();

}

int main(int argc, char **argv) {
    std_msgs::UInt8MultiArray r_buffer;
    MyRobot* pmyData = getMyData();
    ros::init(argc, argv, "/sweepRobotnode");
    //后面的RECORD_IMU变量的主名字
    ros::NodeHandle n("sweepRobotnode");
    ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("/imu0", 1000);
    ros::Publisher main_pub = n.advertise<std_msgs::Bool>("/main_ready", 5);
    ros::Subscriber main_key = n.subscribe("/vo_dir", 10, callbackVO);
    string isIMUrecord = "0";
    //注意添加/，表示ros空间里的变量
    n.param<std::string>("RECORD_IMU", isIMUrecord, "0");
    ROS_INFO("isIMUrecord %s",isIMUrecord.c_str());
    ROS_INFO("sweepRobotnode init.");
    //return 0;
    try {
        ros_ser.setPort("/dev/ttyUSB0");
        ros_ser.setBaudrate(115200); //原460800，现根据扫地机器人说明书调
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
        ROS_INFO_STREAM("Serial Port opened in main");
    }
    else
        return -1;

    int freqSerial = 400;
    int freqControl = 5;
    uint8_t freqCount = 0;
    static uint8_t movingLast = 0;
    // 打开传感器开关 todo 读串口数据
    ros::Rate loop_rate(freqSerial);
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

        if (!pmyData->isAllOn){
            TurnAllSwitch(1);
            pmyData->isAllOn = true;
        }
        if (!pmyData->allSensorEnable){
            AskSensorStatus();
            pmyData->allSensorEnable = true;
        }
        if (!pmyData->sendRegular){
            AskReportRegularly();
            pmyData->sendRegular = true;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    while(vo_dir.empty() && isIMUrecord == "1")
    {
        ROS_ERROR("123");
        ros::spinOnce();
    }
    pmyData->dir = vo_dir;

    std_msgs::Bool mainReady;
    mainReady.data = true;
    main_pub.publish(mainReady);
    ROS_INFO("main ready!");

    uint32 timeLast = 0.0;
    uint32 reportLast = 0;
    double yaw_ = 0.0;
    uint32 reportCount = 0;
    while (ros::ok()) {
        if (ros_ser.available()) {
            std_msgs::UInt8MultiArray serial_data;
            size_t p = ros_ser.available();
            ros_ser.read(serial_data.data, p);
            HandleUART(serial_data.data);

            if (pmyData->imuUpdate) {
                sensor_msgs::Imu imu_data;
                imu_data.header.stamp = ros::Time::now();
                imu_data.header.frame_id = "/sweep";
                //线加速度
                imu_data.linear_acceleration.x = pmyData->accel_x;
                imu_data.linear_acceleration.y = pmyData->accel_y;
                imu_data.linear_acceleration.z = pmyData->accel_z;
                //角速度
                imu_data.angular_velocity.x = pmyData->gyro_x;
                imu_data.angular_velocity.y = pmyData->gyro_y;
                imu_data.angular_velocity.z = pmyData->gyro_z;

                if (isIMUrecord.compare("1")!=0) {
                    //角速度使用去过零漂的
#ifdef ROMOVE_BIAS
                    imu_data.angular_velocity.x = pmyData->w_x_self;
                    imu_data.angular_velocity.y = pmyData->w_y_self;
                    imu_data.angular_velocity.z = pmyData->w_z_self;
#endif
                    imu_data.angular_velocity_covariance[0] = pmyData->odometer_x;
                    imu_data.angular_velocity_covariance[1] = pmyData->odometer_y;
                    imu_data.angular_velocity_covariance[2] = pmyData->odometer_theta;
                    imu_data.angular_velocity_covariance[3] = pmyData->tof;
                    imu_data.angular_velocity_covariance[4] = float(pmyData->pulseLeft);
                    imu_data.angular_velocity_covariance[5] = float(pmyData->pulseRight);
                    imu_data.angular_velocity_covariance[6] = pmyData->roll;
                    imu_data.angular_velocity_covariance[7] = pmyData->pitch;
                    imu_data.angular_velocity_covariance[8] = pmyData->yaw;
                    //ROS_INFO("rosbag record: %d",pmyData->chassisTime);
                }
                //else
                //    ROS_INFO("delta time: %d",pmyData->chassisTime);
                IMU_pub.publish(imu_data);
                pmyData->SaveRobotData(imu_data.header.stamp.toSec(),imu_data.header.stamp.toNSec());
                pmyData->SaveOdometer(pmyData->dir + "./odometry.txt",imu_data.header.stamp.toSec());

                ROS_INFO("Gyro(x,y,z): %f, %f, %f; Acc(x,y,z): %f, %f, %f.",pmyData->gyro_x, pmyData->gyro_y, pmyData->gyro_z, pmyData->accel_x,
                         pmyData->accel_y, pmyData->accel_z);
                //ROS_INFO("delta time: %d,Gyro(x,y,z): %f, %f, %f; Acc(x,y,z): %f, %f, %f; roll %f, pitch %f. yaw %f.",
                //         pmyData->chassisTime - reportLast, pmyData->gyro_x, pmyData->gyro_y, pmyData->gyro_z, pmyData->accel_x,
                //         pmyData->accel_y, pmyData->accel_z, pmyData->roll, pmyData->pitch, pmyData->yaw);
            }
            pmyData->imuUpdate = false;
        }
        //loop_rate.sleep();
    }
}
