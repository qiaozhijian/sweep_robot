//
// Created by qzj on 2020/7/11.
//

#ifndef SRC_IMAGEPRO_H
#define SRC_IMAGEPRO_H

#include "global.h"


Mat addSaltNoise(Mat src, int n);
//给图像添加高斯噪声
double generateGaussianNoise(double mu, double sigma);
Mat addGaussianNoise(Mat& src);

#endif //SRC_IMAGEPRO_H
