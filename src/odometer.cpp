#include "odometer.h"
#include "global.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
using namespace cv;
using namespace Eigen;

nav_msgs::Path path;
void IMUCallback(const sensor_msgs::Imu ::ConstPtr &msg)
{
    Quaterniond quaternion(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);
    Vector3d acc(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
    Vector3d gyro(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    Vector3d odometer(msg->angular_velocity_covariance[0],msg->angular_velocity_covariance[1],msg->angular_velocity_covariance[2]);
    float tof = msg->angular_velocity_covariance[3];
    int pulseLeft = msg->angular_velocity_covariance[4];
    int pulseRight = msg->angular_velocity_covariance[5];

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = odometer(0);
    this_pose_stamped.pose.position.y = odometer(1);


    Eigen::Vector3d eulerAngle(odometer(2),0,0);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternionOdo=yawAngle*pitchAngle*rollAngle;

    this_pose_stamped.pose.orientation.x = quaternionOdo.x();
    this_pose_stamped.pose.orientation.y = quaternionOdo.y();
    this_pose_stamped.pose.orientation.z = quaternionOdo.z();
    this_pose_stamped.pose.orientation.w = quaternionOdo.w();

    this_pose_stamped.header.stamp = msg->header.stamp;
    this_pose_stamped.header.frame_id = "map";
    path.header.frame_id = "sweep";
    path.poses.push_back(this_pose_stamped);

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat frame, frame_L, frame_R;
    try
    {

        cv::Mat stereo = cv_bridge::toCvShare(msg, "bgr8")->image;
        Size dsize = stereo.size();
        frame_L = stereo(Rect(0, 0, dsize.width / 2, dsize.height));  //获取缩放后左Camera的图像
        //imshow("Video_L", frame_L);

        frame_R = stereo(Rect(dsize.width / 2, 0, dsize.width / 2, dsize.height)); //获取缩放后右Camera的图像
        //imshow("Video_R", frame_R);
        //waitKey(0);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    std_msgs::UInt8MultiArray r_buffer;
    ros::init(argc, argv, "odometer_node");
    ros::NodeHandle nh("~");
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory",1, true);
    ros::Subscriber imu_sub = nh.subscribe("robot_imu", 10, IMUCallback);
    ros::Subscriber cam_sub = nh.subscribe("robot_stereo", 10, imageCallback);
    ROS_INFO("odometer_node init.");

    std::string topic_imu;
    std::string topic_camera0, topic_camera1;
    nh.param<std::string>("topic_imu", topic_imu, "/imu0");
    nh.param<std::string>("topic_camera0", topic_camera0, "/cam0/image_raw");
    nh.param<std::string>("topic_camera1", topic_camera1, "/cam1/image_raw");
    // Location of the ROS bag we want to read in
    std::string path_to_bag;
    //nhPrivate.param<std::string>("path_bag", path_to_bag, "/home/keck/catkin_ws/V1_01_easy.bag");
    nh.param<std::string>("path_bag", path_to_bag, "/media/qzj/Document/grow/research/slamDataSet/sweepRobot/round2/03/2020-07-01-21-29-20.bag");
    ROS_INFO(path_to_bag.c_str());
    // Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);
    // Get our start location and how much of the bag we want to play
    // Make the bag duration < 0 to just process to the end of the bag
    double bag_start, bag_durr;
    nh.param<double>("bag_start", bag_start, 0);
    nh.param<double>("bag_durr", bag_durr, -1);
    ROS_INFO("bag start: %.1f",bag_start);
    ROS_INFO("bag duration: %.1f",bag_durr);

    // We should load the bag as a view
    // Here we go from beginning of the bag to the end of the bag
    rosbag::View view_full;
    rosbag::View view;

    // Start a few seconds in from the full view time
    // If we have a negative duration then use the full bag length
    view_full.addQuery(bag);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
    ros::Time time_finish = (bag_durr < 0)? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    ROS_INFO("time start = %.6f", time_init.toSec());
    ROS_INFO("time end   = %.6f", time_finish.toSec());
    view.addQuery(bag, time_init, time_finish);

    // Check to make sure we have data to play
    if (view.size() == 0) {
        ROS_ERROR("No messages to play on specified topics.  Exiting.");
        ros::shutdown();
        return EXIT_FAILURE;
    }

    cv::Mat img0, img1;
    // Step through the rosbag
    for (const rosbag::MessageInstance& m : view) {

        // If ros is wants us to stop, break out
        if (!ros::ok())
            break;

        // Handle IMU measurement
        sensor_msgs::Imu::ConstPtr s2 = m.instantiate<sensor_msgs::Imu>();
        if (s2 != nullptr && m.getTopic() == topic_imu) {
            // convert into correct format
            double timem = (*s2).header.stamp.toSec();
            Eigen::Matrix<double, 3, 1> wm, am;
            wm << (*s2).angular_velocity.x, (*s2).angular_velocity.y, (*s2).angular_velocity.z;
            am << (*s2).linear_acceleration.x, (*s2).linear_acceleration.y, (*s2).linear_acceleration.z;
        }

        // Handle LEFT camera
        sensor_msgs::Image::ConstPtr s0 = m.instantiate<sensor_msgs::Image>();
        if (s0 != nullptr && m.getTopic() == topic_camera0) {
            // Get the image
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(s0);
            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                continue;
            }
            // Save to our temp variable
            img0 = cv_ptr->image.clone();
            namedWindow("img0", 1);
            imshow("img0",img0);
            waitKey(1);
        }

        // Handle RIGHT camera
        sensor_msgs::Image::ConstPtr s1 = m.instantiate<sensor_msgs::Image>();
        if (s1 != nullptr && m.getTopic() == topic_camera1) {
            // Get the image
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(s1);
            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                continue;
            }
            // Save to our temp variable (use a right image that is near in time)
            // TODO: fix this logic as the left will still advance instead of waiting
            // TODO: should implement something like here:
            // TODO: https://github.com/rpng/MARS-VINS/blob/master/example_ros/ros_driver.cpp
            //if(std::abs(cv_ptr->header.stamp.toSec()-time) < 0.02) {
            namedWindow("img1", 1);
            img1 = cv_ptr->image.clone();
            imshow("img1",img1);
            waitKey(1);
            //}
        }
    }
}