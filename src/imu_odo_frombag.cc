/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include <Eigen/Eigen>
#include<ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>
#include<opencv2/core/core.hpp>

using namespace std;

class ImuGrabber {
public:
    ImuGrabber() {};

    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber {
public:
    ImageGrabber(ImuGrabber *pImuGb) : mpImuGb(pImuGb) {
    }

    void GrabImageLeft(const sensor_msgs::ImageConstPtr &msg);

    void GrabImageRight(const sensor_msgs::ImageConstPtr &msg);

    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);

    bool SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft, mBufMutexRight;

    ImuGrabber *mpImuGb;

    bool do_rectify;
    ofstream imu_odo_f;
    cv::Mat M1l, M2l, M1r, M2r;

    //增强对比度
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
static uint32_t imuCnt = 0;
static bool IMUReady = false;
double Td = -0.008;
std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

int main(int argc, char **argv) {

    // Location of the ROS bag we want to read in
//    std::string path_to_bag = "/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/01/2020-07-26-19-47-34.bag";
    //std::string path_to_bag="/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/02/2020-07-26-19-49-21.bag";
    std::string path_to_bag="/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/03/2020-07-26-19-50-56.bag";
//    std::string path_to_bag="/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/04/2020-07-29-18-40-03.bag";
    //std::string path_to_bag="/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/05/2020-07-29-18-41-52.bag";
    //std::string path_to_bag="/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/06/2020-07-29-18-43-57.bag";
    //std::string path_to_bag="/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/07/2020-08-12-16-41-28.bag";
    //std::string path_to_bag="/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/08/2020-08-12-16-47-23.bag";
//    std::string path_to_bag="/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/09/2020-08-12-16-54-51.bag";
//    std::string path_to_bag = "/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round3/10/2020-09-25-13-30-56.bag";

    ROS_INFO("ros bag path is: %s", path_to_bag.c_str());

    // Our camera topics (left and right stereo)
    std::string topic_imu = "/imu0";
    std::string topic_camera0 = "/cam0/image_raw", topic_camera1 = "/cam1/image_raw";

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    double bag_start = 0, bag_durr = -1;
    ROS_INFO("bag start: %.1f", bag_start);
    ROS_INFO("bag duration: %.1f", bag_durr);

    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);

    // We should load the bag as a view
    // Here we go from beginning of the bag to the end of the bag
    rosbag::View view_full;
    rosbag::View view;

    // Start a few seconds in from the full view time
    // If we have a negative duration then use the full bag length
    view_full.addQuery(bag);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
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


    // Step through the rosbag
    for (const rosbag::MessageInstance &m : view) {

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // Handle IMU measurement
        sensor_msgs::Imu::ConstPtr s2 = m.instantiate<sensor_msgs::Imu>();
        if (s2 != nullptr && m.getTopic() == topic_imu) {
            imugb.GrabImu(s2);
        }

        // Handle LEFT camera
        sensor_msgs::Image::ConstPtr s0 = m.instantiate<sensor_msgs::Image>();
        if (s0 != nullptr && m.getTopic() == topic_camera0) {
//            igb.GrabImageLeft(s0);
        }

        // Handle RIGHT camera
        sensor_msgs::Image::ConstPtr s1 = m.instantiate<sensor_msgs::Image>();
        if (s1 != nullptr && m.getTopic() == topic_camera1) {
//            igb.GrabImageRight(s1);
        }
    }

    // save
    uint32_t size_imu = imugb.imuBuf.size();
#define SIZE_EVERY_MSG 14
    float imu_odo[size_imu * SIZE_EVERY_MSG];
    uint64_t timeStamps[size_imu];
    uint64_t cnt = 0;
    while (!imugb.imuBuf.empty()) {
        sensor_msgs::ImuConstPtr imu_msg = imugb.imuBuf.front();
        timeStamps[cnt] = imu_msg->header.stamp.toNSec();
        imu_odo[SIZE_EVERY_MSG * cnt + 0] = imu_msg->angular_velocity.x;
        imu_odo[SIZE_EVERY_MSG * cnt + 1] = imu_msg->angular_velocity.y;
        imu_odo[SIZE_EVERY_MSG * cnt + 2] = imu_msg->angular_velocity.z;
        imu_odo[SIZE_EVERY_MSG * cnt + 3] = imu_msg->linear_acceleration.x;
        imu_odo[SIZE_EVERY_MSG * cnt + 4] = imu_msg->linear_acceleration.y;
        imu_odo[SIZE_EVERY_MSG * cnt + 5] = imu_msg->linear_acceleration.z;
        imu_odo[SIZE_EVERY_MSG * cnt + 6] = imu_msg->angular_velocity_covariance[0];
        imu_odo[SIZE_EVERY_MSG * cnt + 7] = imu_msg->angular_velocity_covariance[1];
        imu_odo[SIZE_EVERY_MSG * cnt + 8] = imu_msg->angular_velocity_covariance[2];
        imu_odo[SIZE_EVERY_MSG * cnt + 9] = imu_msg->angular_velocity_covariance[4];
        imu_odo[SIZE_EVERY_MSG * cnt + 10] = imu_msg->angular_velocity_covariance[5];
        imu_odo[SIZE_EVERY_MSG * cnt + 11] = imu_msg->angular_velocity_covariance[6];
        imu_odo[SIZE_EVERY_MSG * cnt + 12] = imu_msg->angular_velocity_covariance[7];
        imu_odo[SIZE_EVERY_MSG * cnt + 13] = imu_msg->angular_velocity_covariance[8];
        cnt++;
        imugb.imuBuf.pop();
//        cout << e_x << " " << e_y << endl;
//        igb.imu_odo_f << t << " " << dx << " " << dy << " " << dz << " " << rx << " " << ry << " " << rz << " " << e_x << " " << e_y << endl;
    }

    std::ofstream outF("imu_odo.bin", std::ios::binary);
    outF.write(reinterpret_cast<char*>(imu_odo), sizeof(imu_odo));
    outF.close();

    std::ofstream outF2("timeStamp.bin", std::ios::binary);
    outF2.write(reinterpret_cast<char*>(timeStamps), sizeof(timeStamps));
    outF2.close();

    cout << "finish " << size_imu << endl;

    return 0;
}

int imgIdx = 0;
const int speedUp = 1;

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg) {
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d acc(dx, dy, dz);
    Eigen::Vector3d gyr(rx, ry, rz);
    cv::Point2f encoder(imu_msg->angular_velocity_covariance[4],
                        imu_msg->angular_velocity_covariance[5]);

    return;
}

void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg) {
    if (!IMUReady)
        return;
    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    //img_msg->header.stamp = ;
    imgIdx++;
    if (imgIdx % speedUp == 0)
        imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg) {
    if (!IMUReady)
        return;
    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    if (imgIdx % speedUp == 0)
        imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) {
        try {
            cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        if (cv_ptr->image.type() == 16) {
            return cv_ptr->image.clone();
        } else {
            std::cout << "Error type" << std::endl;
            return cv_ptr->image.clone();
        }
    } else if (img_msg->encoding == sensor_msgs::image_encodings::MONO8) {
        try {
            cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        if (cv_ptr->image.type() == 0) {
            return cv_ptr->image.clone();
        } else {
            std::cout << "Error type" << std::endl;
            return cv_ptr->image.clone();
        }
    }
}

