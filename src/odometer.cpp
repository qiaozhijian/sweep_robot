#include "odometer.h"
#include "global.h"
#include "myRobot.h"

using namespace cv;
using namespace Eigen;

nav_msgs::Path path;
bool receiveIMU = false;

void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    Quaterniond quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Vector3d odometer(msg->angular_velocity_covariance[0], msg->angular_velocity_covariance[1],
                      msg->angular_velocity_covariance[2]);
    float tof = msg->angular_velocity_covariance[3];
    int pulseLeft = msg->angular_velocity_covariance[4];
    int pulseRight = msg->angular_velocity_covariance[5];

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = odometer(0);
    this_pose_stamped.pose.position.y = odometer(1);
    this_pose_stamped.pose.position.z = 0.0;


    Eigen::Vector3d eulerAngle(odometer(2), 0, 0);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternionOdo = yawAngle * pitchAngle * rollAngle;

    this_pose_stamped.pose.orientation.x = quaternionOdo.x();
    this_pose_stamped.pose.orientation.y = quaternionOdo.y();
    this_pose_stamped.pose.orientation.z = quaternionOdo.z();
    this_pose_stamped.pose.orientation.w = quaternionOdo.w();

    this_pose_stamped.header.stamp = msg->header.stamp;
    this_pose_stamped.header.frame_id = "/sweep";
    path.header.frame_id = "/sweep";
    path.poses.push_back(this_pose_stamped);
    receiveIMU = true;

    ROS_INFO("IMU gyro_x %f, gyro_y %f, gyro_z %f.", gyro(0), gyro(1), gyro(2));

}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    Mat frame, frame_L, frame_R;
    try {

        cv::Mat stereo = cv_bridge::toCvShare(msg, "bgr8")->image;
        Size dsize = stereo.size();
        frame_L = stereo(Rect(0, 0, dsize.width / 2, dsize.height));  //获取缩放后左Camera的图像
        //imshow("Video_L", frame_L);

        frame_R = stereo(Rect(dsize.width / 2, 0, dsize.width / 2, dsize.height)); //获取缩放后右Camera的图像
        //imshow("Video_R", frame_R);
        //waitKey(0);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char **argv) {
    std_msgs::UInt8MultiArray r_buffer;
    ros::init(argc, argv, "odometer_node");
    ros::NodeHandle nh("~");
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/trajectory", 1, true);
    ros::Subscriber imu_sub = nh.subscribe("/imu0", 10, IMUCallback);
    ros::Subscriber cam_sub = nh.subscribe("robot_stereo", 10, imageCallback);
    ROS_INFO("odometer_node init.");

    MyRobot myRobot(true, false);
    nh.param<std::string>("topic_imu", myRobot.topic_imu, "/imu0");
    nh.param<std::string>("topic_camera0", myRobot.topic_camera0, "/cam0/image_raw");
    nh.param<std::string>("topic_camera1", myRobot.topic_camera1, "/cam1/image_raw");
    nh.param<std::string>("path_bag", myRobot.path_to_bag, "/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round2/06/2020-07-22-13-04-37.bag");
    nh.param<double>("bag_start", myRobot.bag_start, 0);
    nh.param<double>("bag_durr", myRobot.bag_durr, -1);
    ROS_INFO("load bag %s.", myRobot.path_to_bag.c_str());

    ros::Rate rate(100);
    //while (ros::ok())
    //{
    //    if(receiveIMU) {
    //        path_pub.publish(path);
    //        receiveIMU = false;
    //    }
    //    ros::spinOnce();
    //    rate.sleep();
    //}

    myRobot.RosbagInit();

    // Step through the rosbag
    for (const rosbag::MessageInstance &m : myRobot.view) {
        // If ros is wants us to stop, break out
        if (!ros::ok()) {
            ROS_INFO("!ros::ok().");
            break;
        }
        myRobot.HandleRosbag(m);
        //rate.sleep();
    }
    myRobot.odometryFile.close();
    ROS_INFO("odometer_node end.");

}