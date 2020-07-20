//
// Created by qzj on 2020/7/3.
//
#include "global.h"
#include "myRobot.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "imagePro.h"

using namespace cv;
using namespace std;

MyRobot::MyRobot(bool writeOdo, bool writeBag) {
    mWriteOdo = writeOdo;
    mWriteBag = writeBag;
    if (writeOdo) {
        odometryFile = ofstream("./odometry.txt");
    }
    ROS_INFO("MyRobot Created.");

}

void MyRobot::HandleRosbag(rosbag::MessageInstance m) {
    sensor_msgs::Imu::ConstPtr s2 = m.instantiate<sensor_msgs::Imu>();
    if (s2 != nullptr && m.getTopic() == topic_imu) {
        // convert into correct format
        imuTime = (*s2).header.stamp.toSec();
        imuTime = imuTime - timeInit;

        //tf::Quaternion orientation;
        //tf::quaternionMsgToTF(s2->orientation, orientation);
        //tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        //Eigen::Quaterniond qEuler(orientation.w(),orientation.x(),orientation.y(),orientation.z());
        //Eigen::Vector3d eulerAngle=qEuler.matrix().eulerAngles(0,1,2);

        gyro_x = s2->angular_velocity.x;
        gyro_y = s2->angular_velocity.y;
        gyro_z = s2->angular_velocity.z;
        accel_x = s2->linear_acceleration.x;
        accel_y = s2->linear_acceleration.y;
        accel_z = s2->linear_acceleration.z;

        odometer_x = s2->angular_velocity_covariance[0];
        odometer_y = s2->angular_velocity_covariance[1];
        odometer_theta = s2->angular_velocity_covariance[2];
        tof = s2->angular_velocity_covariance[3];
        pulseLeft = s2->angular_velocity_covariance[4];
        pulseRight = s2->angular_velocity_covariance[5];

        if (mWriteOdo) {
            ROS_INFO("mWriteOdo time: %s.", to_string(imuTime).c_str());
            odometryFile << to_string(imuTime) << " " << to_string(gyro_x) << " " << to_string(gyro_y) << " "
                         << to_string(gyro_z) << " " << to_string(accel_x) << " " << to_string(accel_y) << " "
                         << to_string(accel_z) << " " << to_string(pulseLeft) << " " << to_string(pulseRight) << endl;
        }
        //ROS_INFO("IMU time: %f, gyro_x %f, gyro_y %f, gyro_z %f.", imuTime,gyro_x,gyro_y, gyro_z);
        //ROS_INFO("IMU time: %f, roll %f, pitch %f, yaw %f. Odometry x: %f, y %f, theta %f.", imuTime,roll/3.1415926*180.0,pitch/3.1415926*180.0, yaw/3.1415926*180.0, odometer_x,odometer_y,odometer_theta/3.1415926*180.0);
        //ROS_INFO("IMU time: %f, roll %f, pitch %f, yaw %f. Odometry x: %f, y %f, theta %f.", imuTime,eulerAngle(0)/3.1415926*180.0,eulerAngle(1)/3.1415926*180.0, eulerAngle(2)/3.1415926*180.0, odometer_x,odometer_y,odometer_theta/3.1415926*180.0);
    }

    // Handle LEFT camera
    sensor_msgs::Image::ConstPtr s0 = m.instantiate<sensor_msgs::Image>();
    if (s0 != nullptr && m.getTopic() == topic_camera0) {
        // Get the image
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(s0);
            img0Time = (*s0).header.stamp.toSec();
            img0Time = img0Time - timeInit;
            //ROS_INFO(("img0 "+to_string(img0Time)).c_str());
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        // Save to our temp variable
        img0 = cv_ptr->image.clone();
        //namedWindow("img0", 1);
        //imshow("img0",img0);
        //waitKey(1);
    }

    // Handle RIGHT camera
    sensor_msgs::Image::ConstPtr s1 = m.instantiate<sensor_msgs::Image>();
    if (s1 != nullptr && m.getTopic() == topic_camera1) {
        // Get the image
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(s1);
            img1Time = (*s1).header.stamp.toSec();
            img1Time = img1Time - timeInit;
            //ROS_INFO(("img1 "+to_string(img1Time)).c_str());
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        if (std::abs(img0Time - img1Time) < 0.05) {
            //namedWindow("img1", 1);
            img1 = cv_ptr->image.clone();
            //imshow("img1",img1);
            //waitKey(1);
        } else {
            ROS_ERROR("cv_bridge exception: %s %f %f", "stereo inconsistent.", img0Time, img1Time);
        }
    }

}


uint8 MyRobot::UndisImage(rosbag::MessageInstance m) {

    sensor_msgs::Imu::ConstPtr s2 = m.instantiate<sensor_msgs::Imu>();
    if (s2 != nullptr && m.getTopic() == topic_imu) {
        imu_data = *s2;
        return IMU_TOPIC;
    }

    // Handle LEFT camera
    sensor_msgs::Image::ConstPtr s0 = m.instantiate<sensor_msgs::Image>();
    if (s0 != nullptr && m.getTopic() == topic_camera0) {
        // Get the image
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(s0);
            img0Time = (*s0).header.stamp.toSec();
            img0Time = img0Time - timeInit;
            //ROS_INFO(("img0 "+to_string(img0Time)).c_str());
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        // Save to our temp variable
        img0 = cv_ptr->image.clone();
        img0 = addGaussianNoise(img0);

        msg0 = cv_bridge::CvImage(std_msgs::Header(), "mono8", img0).toImageMsg();
        msg0->header.stamp = (*s0).header.stamp;
        msg0->header.frame_id = "sweep";

        imshow("img0", img0);
        waitKey(1);
        return LEFT_TOPIC;
    }

    // Handle RIGHT camera
    sensor_msgs::Image::ConstPtr s1 = m.instantiate<sensor_msgs::Image>();
    if (s1 != nullptr && m.getTopic() == topic_camera1) {
        // Get the image
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(s1);
            img1Time = (*s1).header.stamp.toSec();
            img1Time = img1Time - timeInit;
            //ROS_INFO(("img1 "+to_string(img1Time)).c_str());
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        if (std::abs(img0Time - img1Time) < 0.2) {
            img1 = cv_ptr->image.clone();
        } else {
            ROS_ERROR("cv_bridge exception: %s %f %f", "stereo inconsistent.", img0Time, img1Time);
        }

        img1 = addGaussianNoise(img1);
        imshow("img1", img1);
        waitKey(1);
        msg1 = cv_bridge::CvImage(std_msgs::Header(), "mono8", img1).toImageMsg();
        msg1->header.stamp = (*s1).header.stamp;
        msg1->header.frame_id = "sweep";

        return RIGHT_TOPIC;
    }

}

int MyRobot::RosbagInit() {
    bag.open(path_to_bag, rosbag::bagmode::Read);
    ROS_INFO("bag start: %.1f", bag_start);
    ROS_INFO("bag duration: %.1f", bag_durr);

    // Start a few seconds in from the full view time
    // If we have a negative duration then use the full bag length
    view_full.addQuery(bag);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
    timeInit = time_init.toSec();
    ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    ROS_INFO("time start = %.6f", time_init.toSec());
    ROS_INFO("time end   = %.6f", time_finish.toSec());
    view.addQuery(bag, time_init, time_finish);

    // Check to make sure we have data to play
    if (view.size() == 0) {
        ROS_ERROR("No messages to play on specified topics.  Exiting.");
        ros::shutdown();
        return EXIT_FAILURE;
    }
    ROS_INFO("Init finish");
}

