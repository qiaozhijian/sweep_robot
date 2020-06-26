//
// Created by qzj on 2020/6/24.
//

#ifndef SRC_COMMAND_H
#define SRC_COMMAND_H
#include "global.h"

class SerialData{
public:
    int cnt=0;
    uint32_t chassisTime = 0; //移动底盘的时间

    bool isAllOn = false;
    bool allSensorEnable = false;
    bool sendRegular = false;
    uint8_t moving = 0;

    float tof = 0;// 离墙距离 m
    uint16_t leftCur = 0; //左轮电流
    uint16_t rightCur = 0; //右轮电流
    char leftCharge = 0; //左接收
    char midLeftCharge = 0; //中左接收
    char midRightCharge = 0; //中右接收
    char rightFwdCharge = 0; //右前接收
    char rightCharge = 0; //右接收
    float accel_x = 0.f;
    float accel_y = 0.f;
    float accel_z = 0.f;
    float gyro_x = 0.f;
    float gyro_y = 0.f;
    float gyro_z = 0.f;
    float pitch = 0.f;
    float roll = 0.f;
    float yaw = 0.f;
    int16_t pulseLeft = 0;//左轮脉冲
    int16_t pulseRight = 0;//右轮脉冲
    float odometer_x = 0.f;
    float odometer_y = 0.f;
    float odometer_theta = 0.f;

private:
    int a,b,c;

};

void Move(float vel_ms, float w_rads);
void AskSensorStatus();
void TurnAllSwitch(uint8_t mode);
void AskReportRegularly();
void HandleUART(vector<uint8> data);
void InfoReceive(uint16_t id);
#endif //SRC_COMMAND_H
