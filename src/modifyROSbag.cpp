//
// Created by qzj on 2020/7/11.
//

#include "global.h"
#include "myRobot.h"
#include "imagePro.h"

using namespace cv;
using namespace Eigen;


int main(int argc, char **argv) {
    std_msgs::UInt8MultiArray r_buffer;
    ros::init(argc, argv, "odometer_node");
    ros::NodeHandle nh("~");
    ROS_INFO("odometer_node init.");

    ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("/imu0", 1000);
    image_transport::ImageTransport it(nh);
    //在camera/image话题上发布图像，这里第一个参数是话题的名称，第二个是缓冲区的大小
    image_transport::Publisher pub0 = it.advertise("/cam0/image_raw", 50);
    image_transport::Publisher pub1 = it.advertise("/cam1/image_raw", 50);

    MyRobot myRobot(false, true);
    nh.param<std::string>("topic_imu", myRobot.topic_imu, "/imu0");
    nh.param<std::string>("topic_camera0", myRobot.topic_camera0, "/cam0/image_raw");
    nh.param<std::string>("topic_camera1", myRobot.topic_camera1, "/cam1/image_raw");
    nh.param<std::string>("path_bag", myRobot.path_to_bag,
                          "/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round2/cali/camera_imu/camera_imu/try.bag");
    ROS_INFO("load %s", myRobot.path_to_bag.c_str());
    nh.param<double>("bag_start", myRobot.bag_start, 0);
    nh.param<double>("bag_durr", myRobot.bag_durr, -1);

    ros::Rate rate(100);

    myRobot.RosbagInit();

    // Step through the rosbag
    for (const rosbag::MessageInstance &m : myRobot.view) {
        // If ros is wants us to stop, break out
        if (!ros::ok())
            break;
        if (myRobot.UndisImage(m) == IMU_TOPIC) {
            IMU_pub.publish(myRobot.imu_data);
        } else if (myRobot.UndisImage(m) == LEFT_TOPIC) {
            pub0.publish(myRobot.msg0);
        } else if (myRobot.UndisImage(m) == RIGHT_TOPIC) {
            pub1.publish(myRobot.msg1);
        }
        //rate.sleep();
    }
    if (myRobot.mWriteOdo)
        myRobot.odometryFile.close();
    ROS_INFO("odometer_node end.");

}
