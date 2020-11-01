//
// Created by qzj on 2020/7/11.
//

#include "imagePro.h"


Mat addSaltNoise(Mat src, int n) {
    Mat result = src.clone();
    for (int k = 0; k < n; k++) {
        //随机选取行列值
        int i = rand() % result.cols;
        int j = rand() % result.rows;
        if (result.channels() == 1) {
            result.at<uchar>(j, i) = 255;
        } else {
            result.at<Vec3b>(j, i)[0] = 255;
            result.at<Vec3b>(j, i)[1] = 255;
            result.at<Vec3b>(j, i)[2] = 255;
        }

    }
    return result;
}

//给图像添加高斯噪声
double generateGaussianNoise(double mu, double sigma) {
    //定义最小值
    double epsilon = numeric_limits<double>::min();
    double z0 = 0, z1 = 0;
    bool flag = false;
    flag = !flag;
    if (!flag)
        return z1 * sigma + mu;
    double u1, u2;
    do {
        u1 = rand() * (1.0 / RAND_MAX);
        u2 = rand() * (1.0 / RAND_MAX);
    } while (u1 <= epsilon);
    z0 = sqrt(-2.0 * log(u1)) * cos(2 * CV_PI * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(2 * CV_PI * u2);
    return z0 * sigma + mu;
}

Mat addGaussianNoise(Mat &src) {
    Mat result = src.clone();
    int channels = result.channels();
    int nRows = result.rows;
    int nCols = result.cols * channels;
    if (result.isContinuous()) {
        nCols = nCols * nRows;
        nRows = 1;
    }
    for (int i = 0; i < nRows; i++) {
        for (int j = 0; j < nCols; j++) {
            int val = result.ptr<uchar>(i)[j] + generateGaussianNoise(2, 0.8) * 32;
            if (val < 0)
                val = 0;
            if (val > 255)
                val = 255;
            result.ptr<uchar>(i)[j] = (uchar) val;
        }
    }
    return result;
}


