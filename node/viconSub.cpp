//
// Created by qzj on 2020/7/26.
//

#include "viconSub.h"
#include "global.h"
#include "myRobot.h"
#include "geometry_msgs/TransformStamped.h"

using namespace cv;
using namespace Eigen;

ofstream viconFile;

void VICON_Callback(const geometry_msgs::TransformStamped &msg) {
    double t = msg.header.stamp.toSec();
    Vector3d trans(msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z);
    Vector4d quat(msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w);

    viconFile<<to_string(t)<<" ";
    viconFile<<to_string(trans(0))<<" "<<to_string(trans(1))<<" "<<to_string(trans(2))<<" ";
    viconFile<<to_string(quat(0))<<" "<<to_string(quat(1))<<" "<<to_string(quat(2))<<" "<<to_string(quat(3));
    viconFile<<endl;
}


int main(int argc, char **argv) {
    std_msgs::UInt8MultiArray r_buffer;
    ros::init(argc, argv, "viconSub_node");
    ros::NodeHandle nh("~");
    ros::Subscriber cam_sub = nh.subscribe("/vicon/sweeper/sweeper", 10, VICON_Callback);
    ROS_INFO("viconSub_node init.");

    time_t now_time = time(NULL);
    tm *T_tm = localtime(&now_time);
    string timeDetail = asctime(T_tm);
    timeDetail.pop_back();
    string dir = "./vicon-" + timeDetail + ".txt";
    viconFile = ofstream(dir);
    ros::spin();
    viconFile.close();
    ROS_INFO("viconSub_node finish");
}

