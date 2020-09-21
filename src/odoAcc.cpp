#include "odometer.h"
#include "global.h"
#include "myRobot.h"

using namespace cv;
using namespace Eigen;

static bool isInitial = false;
ofstream f;
void calAccOdo(double& t, const Vector2d& encoder, Vector3d& accOdo, Vector2d& odoSelf, const Vector3d& rpy)
{
    static Vector2d encoderLast=Vector2d::Zero(), odoSelfLast=Vector2d::Zero();
    static double timeLast=0.0,yawLast=0.0;
    static vector<pair<double, pair<double, double>>> delta_t_X;

    Vector2d delatEncoder = encoder - encoderLast;
    if(!isInitial)
    {
        delatEncoder = Eigen::Vector2d::Zero();
    } else{
        if(delatEncoder(0) >= (5.0 * 18.0*67.2))
            delatEncoder(0) = delatEncoder(0) - 10.0 * 18.0*67.2;
        else if(delatEncoder(0) <= -(5.0 * 18.0*67.2))
            delatEncoder(0) = delatEncoder(0) + 10.0 * 18.0*67.2;
        if(delatEncoder(1) >= (5.0 * 18.0*67.2))
            delatEncoder(1) = delatEncoder(1) - 10.0 * 18.0*67.2;
        else if(delatEncoder(1) <= -(5.0 * 18.0*67.2))
            delatEncoder(1) = delatEncoder(1) + 10.0 * 18.0*67.2;
    }
    //delatEncoder = delatEncoder/(18.0*67.2)*M_PI*0.07;
    delatEncoder = delatEncoder * 0.0001818051304160764;
    double delatEncoderMid=0.0, curV=0.0;
    delatEncoderMid = (delatEncoder(0) + delatEncoder(1)) / 2.0;
    double deltaT = 0.0;
    if(isInitial)
    {
        deltaT = t - timeLast;
        curV = delatEncoderMid / deltaT;
    }

    if(delta_t_X.size()<6)
        delta_t_X.push_back(make_pair(deltaT, make_pair(delatEncoderMid,curV)));
    else{
        delta_t_X.erase(delta_t_X.begin());
        delta_t_X.push_back(make_pair(deltaT, make_pair(delatEncoderMid,curV)));
    }
    accOdo = Vector3d::Zero();
    if(delta_t_X.size()==6)
    {
        vector<double > acc(5);
        for(int i = 0;i<5;i++)
            acc.at(i) = (delta_t_X.at(i+1).second.second - delta_t_X.at(i).second.second)/(delta_t_X.at(i+1).first + delta_t_X.at(i).first)/2.0;
        Vector3d acc3;
        for(int i=0;i<3;i++)
            acc3(i) = (acc.at(i) + acc.at(i+1) +acc.at(i+2))/3.0;
        accOdo(0) =(acc3(0) + acc3(1) +acc3(2))/3.0;
        //accOdo(0) = acc.at(4);
    }

    Vector2d delatOdo = Vector2d::Zero();
    double rpyRad = rpy(2) * M_PI / 180.0;
    if(isInitial)
    {
        //欧拉中值积分
        double yawCur = (rpyRad+ yawLast) / 2.0;
        delatOdo(0) = delatEncoderMid * cos(yawCur);
        delatOdo(1) = delatEncoderMid * sin(yawCur);

        // 李群上积分
        Matrix2d R;
        R<<cos(yawLast),-sin(yawLast),sin(yawLast),cos(yawLast);
        Matrix2d A;
        double alpha = rpyRad - yawLast;
        if(abs(alpha)>0.0001)
            A<<sin(alpha)/alpha,-(1-cos(alpha))/alpha,(1-cos(alpha))/alpha,sin(alpha)/alpha;
        else
            A<<cos(alpha),-(0 + sin(alpha))/1,(0 + sin(alpha))/1,cos(alpha);
        Vector2d v = Vector2d::Zero();
        v(0) = delatEncoderMid;
        delatOdo = R*A*v;
    }

    odoSelf = odoSelfLast + delatOdo;

    yawLast = rpyRad;
    odoSelfLast = odoSelf;
    encoderLast = encoder;
    timeLast = t;
}
template<typename T>
Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1>& vec)
{
    return (Eigen::Matrix<T, 3, 3>() << T(0), -vec(2), vec(1),
            vec(2), T(0), -vec(0),
            -vec(1), vec(0), T(0)).finished();
}

void transFormAcc(Vector3d& accOdo, const Vector3d& rpy, const Vector3d& turnRate)
{
    Vector3d accIMU;
    Matrix4d T_imu_odo;
    T_imu_odo<<-1,0,0,0.133,0,-1,0,0,0,0,1,-0.02564,0,0,0,1;
    Matrix4d T_odo_imu = T_imu_odo.inverse();

    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;

    Matrix3d turnRateSkew = skew(turnRate);

    Matrix4d T_odoAcc = Matrix4d::Identity();
    T_odoAcc.block(0,0,3,3) = quaternion.matrix() * turnRateSkew * turnRateSkew;
    T_odoAcc.block(0,3,3,1) = accOdo;


    T_odoAcc = T_imu_odo * T_odoAcc * T_odo_imu;

    accOdo = T_odoAcc.block(0,3,3,1);

}
void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    double time_now = msg->header.stamp.toSec();
    static Vector3d accFirst = Vector3d::Zero();

    Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Vector3d odometer(msg->angular_velocity_covariance[0], msg->angular_velocity_covariance[1],
                      msg->angular_velocity_covariance[2]);
    Vector2d encoder(msg->angular_velocity_covariance[4],
                      msg->angular_velocity_covariance[5]);
    Vector3d rpy(msg->angular_velocity_covariance[6], msg->angular_velocity_covariance[7],
                      msg->angular_velocity_covariance[8]);

    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;

    Vector2d odoSelf;
    Vector3d accOdo;

    calAccOdo(time_now, encoder, accOdo, odoSelf, rpy);
    transFormAcc(accOdo, rpy, gyro );
    if(!isInitial)
        accFirst = acc - accOdo;
    accOdo = accOdo + accFirst;

    f << fixed;
    f  << time_now << " " << acc(0) << " " << acc(1) << " " << acc(2) << " " << accOdo(0) << " " << accOdo(1) << " " << accOdo(2) << endl;

    ROS_INFO("self x %f, y %f; usat: x %f, y %f, acc_x %f, acc %f.", odoSelf(0), odoSelf(1), odometer(0), odometer(1), acc(0), accOdo(0));
    if(!isInitial)
        isInitial = true;
}


int main(int argc, char **argv) {
    std_msgs::UInt8MultiArray r_buffer;
    ros::init(argc, argv, "odoAcc_node");
    ros::NodeHandle nh("~");
    string filename = "./acc.txt";
    //f.open(filename.c_str(), ios::out | ios::app);
    f.open(filename.c_str(), ios::out);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/trajectory", 1, true);
    ros::Subscriber imu_sub = nh.subscribe("/imu0", 10, IMUCallback);
    ROS_INFO("odoAcc_node init.");

    MyRobot myRobot(true, false);
    nh.param<std::string>("topic_imu", myRobot.topic_imu, "/imu0");
    nh.param<std::string>("topic_camera0", myRobot.topic_camera0, "/cam0/image_raw");
    nh.param<std::string>("topic_camera1", myRobot.topic_camera1, "/cam1/image_raw");

    ros::spin();

    f.close();
    printf("odometer_node end.");

}