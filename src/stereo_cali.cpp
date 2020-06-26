#include<iostream>
#include<string>
#include<sstream>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/videoio.hpp>
#include<opencv2/opencv.hpp>
#include<stdio.h>

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
#define ACCESS(fileName,accessMode) access(fileName,accessMode)
#define MKDIR(path) mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif

int32_t createDirectory(const std::string &directoryPath)
{
    uint32_t dirPathLen = directoryPath.length();
    if (dirPathLen > MAX_PATH_LEN)
    {
        return 1;
    }
    char tmpDirPath[MAX_PATH_LEN] = { 0 };
    for (uint32_t i = 0; i < dirPathLen; ++i)
    {
        tmpDirPath[i] = directoryPath[i];
        if (tmpDirPath[i] == '\\' || tmpDirPath[i] == '/')
        {
            if (ACCESS(tmpDirPath, 0) != 0)
            {
                int32_t ret = MKDIR(tmpDirPath);
                if (ret != 0)
                {
                    return ret;
                }
            }
        }
    }
    return 0;
}

using namespace std;
using namespace cv;
//
int main(int argc, const char **argv)            //程序主函数
{
    VideoCapture cap;
    int width = 2560;
    int height = 720;
    int FPS = 50;
    cap.open(2);                             //打开相机，电脑自带摄像头一般编号为0，外接摄像头编号为1，主要是在设备管理器中查看自己摄像头的编号。
    //--------------------------------------------------------------------------------------
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));//视频流格式
    cap.set(CV_CAP_PROP_FPS, FPS);//帧率
    cap.set(CV_CAP_PROP_FRAME_WIDTH, width);  //设置捕获视频的宽度
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);  //设置捕获视频的高度

    if (!cap.isOpened())                         //判断是否成功打开相机
    {
        cout << "摄像头打开失败!" << endl;
        return -1;
    }
    Mat frame, frame_L, frame_R;

    cap >> frame;                                //从相机捕获一帧图像

    Mat grayImage;                               //用于存放灰度数据

    double fScale = 1;                         //定义缩放系数，对2560*720图像进行缩放显示（2560*720图像过大，液晶屏分辨率较小时，需要缩放才可完整显示在屏幕）
    Size dsize = Size(frame.cols * fScale, frame.rows * fScale);
    Mat imagedst = Mat(dsize, CV_32S);
    resize(frame, imagedst, dsize);
    char key;
    char image_left[200];
    char image_right[200];
    int count = 0;
    createDirectory("./img/right/");
    createDirectory("./img/left/");
    while (1) {
        cap >> frame;                            //从相机捕获一帧图像
        resize(frame, imagedst, dsize);          //对捕捉的图像进行缩放操作

        frame_L = imagedst(Rect(0, 0, dsize.width/2, dsize.height));  //获取缩放后左Camera的图像
        namedWindow("Video_L", 1);
        imshow("Video_L", frame_L);

        frame_R = imagedst(Rect(dsize.width/2, 0, dsize.width/2, dsize.height)); //获取缩放后右Camera的图像
        namedWindow("Video_R", 1);
        imshow("Video_R", frame_R);

        //按下ESC退出
        if (key == 27)
            break;
        //32对应空格
        if (key == 32) {
            sprintf(image_left, "./img/left/%06d.jpg", count);
            imwrite(image_left, frame_L);
            sprintf(image_right, "./img/right/%06d.jpg", count);
            imwrite(image_right, frame_R);
            count++;
            cout<<"save "<<count<<endl;
        }
        key = waitKey(1);
    }

    return 0;
}
