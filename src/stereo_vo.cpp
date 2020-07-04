#include "global.h"

#ifdef WIN32
#include <io.h>
#include <direct.h>
#else

#include <unistd.h>
#include <sys/stat.h>

#endif
#define MAX_PATH_LEN 256
#ifdef WIN32
#define ACCESS(fileName,accessMode) _access(fileName,accessMode)
#define MKDIR(path) _mkdir(path)
#else
#define ACCESS(fileName, accessMode) access(fileName,accessMode)
#define MKDIR(path) mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif


using namespace std;
using namespace cv;

int32_t createDirectory(const std::string &directoryPath) {
    uint32_t dirPathLen = directoryPath.length();
    if (dirPathLen > MAX_PATH_LEN) {
        return 1;
    }
    char tmpDirPath[MAX_PATH_LEN] = {0};
    for (uint32_t i = 0; i < dirPathLen; ++i) {
        tmpDirPath[i] = directoryPath[i];
        if (tmpDirPath[i] == '\\' || tmpDirPath[i] == '/') {
            if (ACCESS(tmpDirPath, 0) != 0) {
                int32_t ret = MKDIR(tmpDirPath);
                if (ret != 0) {
                    return ret;
                }
            }
        }
    }
    return 0;
}

VideoCapture cap;
int width = 1280;
int height = 480;
int FPS = 50;
string dir;

void createDir() {
    time_t now_time = time(NULL);
    tm *t_tm = localtime(&now_time);
    //转换为年月日星期时分秒结果，如图：
    string timeDetail = asctime(t_tm);
    dir = "./dataset/" + timeDetail + "img/";
    createDirectory(dir + "right/");
    createDirectory(dir + "left/");
}

void InitCap() {
    cap.open(2);                             //打开相机，电脑自带摄像头一般编号为0，外接摄像头编号为1，主要是在设备管理器中查看自己摄像头的编号。
    //--------------------------------------------------------------------------------------
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));//视频流格式
    cap.set(CV_CAP_PROP_FPS, FPS);//帧率
    cap.set(CV_CAP_PROP_FRAME_WIDTH, width);  //设置捕获视频的宽度
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);  //设置捕获视频的高度

    if (!cap.isOpened())                         //判断是否成功打开相机
    {
        cout << "摄像头打开失败!" << endl;
        return;
    }
}

bool saveImages = true;

//
int main(int argc, char **argv)            //程序主函数
{
    ros::init(argc, argv, "stereo_vo_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    //在camera/image话题上发布图像，这里第一个参数是话题的名称，第二个是缓冲区的大小
    image_transport::Publisher pub0 = it.advertise("/cam0/image_raw", 50);
    image_transport::Publisher pub1 = it.advertise("/cam1/image_raw", 50);
    ROS_INFO("stereo_vo_node init.");

    InitCap();
    createDir();

    Mat frame = Mat::zeros(Size(width, height), CV_8UC3);
    Mat frame_L, frame_R;
    char image_idx[200];
    int count = 0;

    ros::Rate loop_rate(200);
    while (ros::ok()) {
        //好像固定50 fps
        if (cap.read(frame)) {

            frame_L = frame(Rect(0, 0, width / 2, height));  //获取缩放后左Camera的图像
            //namedWindow("Video_L", 1);
            //imshow("Video_L", frame_L);

            frame_R = frame(Rect(width / 2, 0, width / 2, height)); //获取缩放后右Camera的图像
            //namedWindow("Video_R", 1);
            //imshow("Video_R", frame_R);

            sprintf(image_idx, "%06d.jpg", count);
            //imwrite(dir + "left/" + image_idx, frame_L);
            //imwrite(dir + "right/" + image_idx, frame_R);
            count++;
            //ROS_INFO("save %d", count);

            sensor_msgs::ImagePtr msg0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_L).toImageMsg();
            msg0->header.stamp = ros::Time::now();
            msg0->header.frame_id = "sweep";
            pub0.publish(msg0);
            sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_R).toImageMsg();
            msg1->header.stamp = ros::Time::now();
            msg1->header.frame_id = "sweep";
            pub1.publish(msg1);
        } else {
            ROS_INFO("fail");
        }
        //waitKey(0);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
