//
// Created by qzj on 2020/6/24.
//

#include "command.h"
#include "myRobot.h"
#include "crc.h"

serial::Serial ros_ser;
MyRobot myData(false, false);

#define RAD_TO_ANGLE 57.295779515
#define STATIC_BUFFER_NUM 500
#define STATIC_JUDGE_NUM 20
#define STATIC_THRESHOLD 0.3

uint8 CheckStatic(double wz) {
    static uint32 staticNum = 0;
    wz = fabs(wz * RAD_TO_ANGLE);
    //ROS_INFO("%f",wz);
    if (wz < STATIC_THRESHOLD)
        staticNum++;
    else
        staticNum = 0;
    if (staticNum > STATIC_JUDGE_NUM) {
        staticNum = STATIC_JUDGE_NUM + 1;
        return 1;
    } else
        return 0;
}

// 输入原始角速度wRaw，输出去零飘的角速度wNew
void RemoveBias(double wRaw[3], double wNew[3]) {
    static double wRawSlide[3][STATIC_BUFFER_NUM] = {0.f};
    static uint32 staticCount = 0;
    static double wBiasSum[3] = {0.f};
    static uint32 staticCountLast = 0;

    if (staticCount >= STATIC_BUFFER_NUM)
        for (uint8 axis = 0; axis < 3; axis++)
            wNew[axis] = wRaw[axis] - wBiasSum[axis] / double(STATIC_BUFFER_NUM);
    else
        for (uint8 axis = 0; axis < 3; axis++)
            wNew[axis] = wRaw[axis];

    if (CheckStatic(wNew[2])) {
        staticCount++;
        if (staticCount > STATIC_BUFFER_NUM)
            staticCount = STATIC_BUFFER_NUM;
        for (uint8 axis = 0; axis < 3; axis++) {
            wBiasSum[axis] = wBiasSum[axis] - wRawSlide[axis][0];
            for (int i = 0; i < STATIC_BUFFER_NUM - 1; i++)
                wRawSlide[axis][i] = wRawSlide[axis][i + 1];
            wRawSlide[axis][STATIC_BUFFER_NUM - 1] = wRaw[axis];
            wBiasSum[axis] = wBiasSum[axis] + wRawSlide[axis][STATIC_BUFFER_NUM - 1];
        }
    } else
        staticCount = 0;

    if (staticCount > 0 && staticCountLast == 0)
        ROS_INFO("robot is static");
    else if (staticCount == 0 && staticCountLast != 0)
        ROS_INFO("robot start to move");
    if (staticCount == STATIC_BUFFER_NUM && staticCountLast != STATIC_BUFFER_NUM)
        ROS_INFO("imu bais update");
    staticCountLast = staticCount;

    for (uint8 axis = 0; axis < 3; axis++)
        if (fabs(wNew[axis] * RAD_TO_ANGLE) < STATIC_THRESHOLD)
            wNew[axis] = 0.0;
}

uint16 CRC16_CCITT_FALSE(uint8 *puchMsg, unsigned int usDataLen) {
    uint16 wCRCin = 0xFFFF; //初始值为 0xFFFF
    uint16 wCPoly = 0x1021; //多项式 x1 6+x1 2+x5+1
    uint8 wChar = 0;
    while (usDataLen--) {
        wChar = *(puchMsg++);
        wCRCin ^= (wChar << 8);
        for (int i = 0; i < 8; i++) {
            if (wCRCin & 0x8000) {
                wCRCin = (wCRCin << 1) ^ wCPoly;
            } else {
                wCRCin = wCRCin << 1;
            }
        }
    }
    return wCRCin;
}
//a5 a5 0 1e 长度 85 58 序号 3 1 命令  a8 1
// 0 64
// fe c2
// 26 62
// 0 0
// ff ff
// ff ff
// 0 0
// 0 0
// 0 0 d 0
// d2 3e   5a 5a

float ToFloat(vector<uint8_t> body, uint8_t shiftMask, uint8_t shiftData) {
    union {
        float angleVelocity;
        uint8 units[4];
    } floatBytes;
    for (uint8 i = 0; i < 4; i++)
        floatBytes.units[4 - i - 1] = body[shiftMask + shiftData + i];
    return floatBytes.angleVelocity;
}

void Analyse(vector<uint8> frame) {
    static uint32 timeLast = 0;
    uint16_t frameLen = CAT(frame[0], frame[1]);
    //减去 长度，帧序号，命令字，校验码
    uint16_t bodyLen = frameLen - 8;
    uint16_t frameCnt = CAT(frame[2], frame[3]);
    uint16_t id = CAT(frame[4], frame[5]);
    vector<uint8_t> body(frame.begin() + 6, frame.begin() + 6 + bodyLen);

    //cout<<setbase(16)<<int(id)<<endl;
    switch (id) {
        case 0x8200:
            if (body[0] == 0x00) {
                if (CAT(body[1], body[2]) == 0x0201) {
                    myData.isAllOn = true;
                    InfoReceive(0x0201);
                } else if (CAT(body[1], body[2]) == 0x0206) {
                    myData.sendRegular = true;
                    InfoReceive(0x0206);
                }
            }
            break;
        case 0x8102:
            if (CAT(body[0], body[1]) == 0xffff)
                myData.allSensorEnable = true;
            break;
        case 0x0301:
            uint8_t shiftMask = 1;
            uint8_t shiftData = 0;
            if (BIT_7(body[0]))
                shiftMask = 2;
            //TOF数据存在
            if (BIT_0(body[0])) {
                myData.tof = body[shiftMask + shiftData] / 1000.0;
                shiftData = shiftData + 1;
                //ROS_INFO("read tof %d %f",shiftMask + shiftData-1,myData.tof);
            }
            //电流数据存在
            if (BIT_1(body[0])) {
                //ROS_INFO("read current");
                myData.leftCur = int16_t(CAT(body[shiftMask + shiftData], body[shiftMask + shiftData + 1]));
                shiftData = shiftData + 2;
                myData.rightCur = int16_t(CAT(body[shiftMask + shiftData], body[shiftMask + shiftData + 1]));
                shiftData = shiftData + 2;
            }
            //回冲数据存在
            if (BIT_2(body[0])) {
                //ROS_INFO("read charge");
                myData.leftCharge = body[shiftMask + shiftData];
                shiftData = shiftData + 1;
                myData.midLeftCharge = body[shiftMask + shiftData];
                shiftData = shiftData + 1;
                myData.midRightCharge = body[shiftMask + shiftData];
                shiftData = shiftData + 1;
                myData.rightFwdCharge = body[shiftMask + shiftData];
                shiftData = shiftData + 1;
                myData.rightCharge = body[shiftMask + shiftData];
                shiftData = shiftData + 1;
            }
            //IMU数据存在
            if (BIT_3(body[0])) {
                //ROS_INFO("read IMU");
                myData.accel_y = int16_t(CAT(body[shiftMask + shiftData], body[shiftMask + shiftData + 1])) / 1000.0;
                shiftData = shiftData + 2;
                myData.accel_x = int16_t(CAT(body[shiftMask + shiftData], body[shiftMask + shiftData + 1])) / 1000.0;
                shiftData = shiftData + 2;
                myData.accel_z = -int16_t(CAT(body[shiftMask + shiftData], body[shiftMask + shiftData + 1])) / 1000.0;
                shiftData = shiftData + 2;
                myData.gyro_y = ToFloat(body, shiftMask, shiftData);
                shiftData = shiftData + 4;
                myData.gyro_x = ToFloat(body, shiftMask, shiftData);
                shiftData = shiftData + 4;
                myData.gyro_z = -ToFloat(body, shiftMask, shiftData);
                shiftData = shiftData + 4;
#ifdef ROMOVE_BIAS
                double gyro_raw[3] = {myData.gyro_x, myData.gyro_y, myData.gyro_z};
                double gyro_new[3] = {0.0};
                RemoveBias(gyro_raw, gyro_new);
                myData.w_x_self = gyro_new[0];
                myData.w_y_self = gyro_new[1];
                myData.w_z_self = gyro_new[2];
                //ROS_INFO("read imu %d %f %f %f %f %f %f",shiftMask + shiftData,myData.accel_x,myData.accel_y,myData.accel_z,myData.gyro_x,myData.gyro_y,myData.gyro_z);
#endif
                myData.imuUpdate = true;
            }
            //姿态数据存在
            if (BIT_4(body[0])) {
                //ROS_INFO("read posture");
                myData.pitch = int16_t(CAT(body[shiftMask + shiftData], body[shiftMask + shiftData + 1])) / 100.0;
                shiftData = shiftData + 2;
                myData.roll = int16_t(CAT(body[shiftMask + shiftData], body[shiftMask + shiftData + 1])) / 100.0;
                shiftData = shiftData + 2;
                myData.yaw = - int16_t(CAT(body[shiftMask + shiftData], body[shiftMask + shiftData + 1])) / 100.0;
                shiftData = shiftData + 2;
                //ROS_INFO("read euler %d %f %f %f.",shiftMask + shiftData,myData.pitch,myData.roll,myData.yaw);
            }
            //轮子脉冲数据存在
            if (BIT_5(body[0])) {
                myData.pulseLeft = int16_t(CAT(body[shiftMask + shiftData], body[shiftMask + shiftData + 1]));
                shiftData = shiftData + 2;
                myData.pulseRight = int16_t(CAT(body[shiftMask + shiftData], body[shiftMask + shiftData + 1]));
                shiftData = shiftData + 2;
                //ROS_INFO("read pulse %d %d %d.",shiftMask + shiftData,myData.pulseLeft,myData.pulseRight);
            }
            //轮式里程计数据存在
            if (BIT_6(body[0])) {
                //ROS_INFO("read dometer");
                myData.odometer_x = int16_t(CAT(body[shiftMask + shiftData], body[shiftMask + shiftData + 1])) / 1000.0;
                shiftData = shiftData + 2;
                myData.odometer_y = int16_t(CAT(body[shiftMask + shiftData], body[shiftMask + shiftData + 1])) / 1000.0;
                shiftData = shiftData + 2;
                myData.odometer_theta =
                        int16_t(CAT(body[shiftMask + shiftData], body[shiftMask + shiftData + 1])) / 10000.0;
                shiftData = shiftData + 2;
                //ROS_INFO("read dometer %d %f %f %f.",shiftMask + shiftData,myData.odometer_x,myData.odometer_y,myData.odometer_theta);
            }
            myData.chassisTime = CAT32(body[shiftMask + shiftData], body[shiftMask + shiftData + 1],
                                       body[shiftMask + shiftData + 2], body[shiftMask + shiftData + 3]);

            shiftData = shiftData + 4;
            if ((shiftData + shiftMask) != bodyLen)
                ROS_INFO("fail to read over %d, %d, %d", shiftData, shiftMask, bodyLen);
            //ROS_INFO("frameCnt %d, delta time: %d, time %d lastTime %d.", frameCnt, myData.chassisTime-timeLast, myData.chassisTime, timeLast);
            timeLast = myData.chassisTime;
            //ROS_INFO("IMU time: %d, roll %f, pitch %f, yaw %f, theta %f.", myData.chassisTime,
            //         myData.roll / 3.1415926 * 180.0, myData.pitch / 3.1415926 * 180.0, myData.yaw / 3.1415926 * 180.0,
            //         myData.odometer_theta / 3.1415926 * 180.0);
            break;
    }
}


void HandleUART(vector<uint8> data) {
    static vector<uint8> curFrame;
    curFrame.reserve(200);
    static uint8_t iterLast = 0;
    static bool reading = false;
    //cout << hex << endl;
    //cout<<"read: ";
    //for(int i=0;i<data.size();i++)
    //    cout<<setbase(16)<<int(data[i])<<" ";
    //cout<<endl;

    for (auto iter:data) {
        if (iter == 0xa5 && iterLast == 0xa5) {
            reading = true;
            curFrame.clear();
            curFrame.resize(0);
            iterLast = iter;
            continue;
        }

        if (iter == 0x5a && iterLast == 0x5a && reading) {
            //判断长度是否满足(字节漏掉的情况是很少的，防止数据位最后一位也是0x5a以至于提前结束)
            if (CAT(curFrame[0], curFrame[1]) + 1 == curFrame.size()) {
                //ROS_INFO("success");
                reading = false;

                //去除尾部0x5a
                curFrame.pop_back();
                Analyse(curFrame);
                curFrame.clear();
                curFrame.resize(0);
            }
            //else
            //    ROS_INFO("fail");
        }

        if (reading)
            curFrame.push_back(iter);

        iterLast = iter;
    }
}


// a5 a5 00 08 00 c8 01 02 90 18 5a 5a   		//获取传感器使能状态
void AskSensorStatus() {
    UART_TYPE08 data = {0};
    data.startCode = SWOP(0xa5a5);
    data.len = SWOP(0x0008);
    data.cnt = SWOP(myData.cnt);
    data.id = SWOP(0x0102);
    uint16 check = CRC16_CCITT_FALSE(data.units + 2, 6);
    data.checkCode = SWOP(check);
    data.endCode = SWOP(0x5a5a);
    //cout << hex << check << endl;
    ros_ser.write(data.units, 12);
    //cout<<"write: ";
    //for(int i=0;i<12;i++)
    //    cout<<setbase(16)<<int(data.units[i])<<" ";
    //cout<<endl;
    myData.cnt++;
}


//A5 A5 00 0A 00 D0 02 01 FF FF 32 C5 5A 5A	//开所有传感器开关
//A5 A5 00 0A 02 CD 02 01 00 00 81 AA 5A 5A	//关所有传感器开关

void TurnAllSwitch(uint8_t mode) {
    UART_TYPE0A data = {0};
    data.startCode = SWOP(0xa5a5);
    data.len = SWOP(0x000a);
    data.cnt = SWOP(myData.cnt);
    data.id = SWOP(0x0201);
    if (mode == 0)
        data.body = SWOP(0x0000);
    else
        data.body = SWOP(0xffff);

    uint16 check = CRC16_CCITT_FALSE(data.units + 2, 8);
    data.checkCode = SWOP(check);
    data.endCode = SWOP(0x5a5a);
    ros_ser.write(data.units, 14);
    myData.cnt++;
}


void InfoReceive(uint16_t id) {
    UART_TYPE0B data = {0};
    data.startCode = SWOP(0xa5a5);
    data.len = SWOP(0x000b);
    data.cnt = SWOP(myData.cnt);
    data.id = SWOP(0x8400);
    data.body1 = 0x00;
    data.body2 = id;
    uint16 check = CRC16_CCITT_FALSE(data.units + 2, 9);
    data.checkCode = SWOP(check);
    data.endCode = SWOP(0x5a5a);
    ros_ser.write(data.units, 13);
    myData.cnt++;
}

void Move(float vel_ms, float w_rads) {
    int16_t vel = int16_t(vel_ms * 1000.0);
    int16_t w = int16_t(w_rads * 1000.0);
    UART_TYPE0C data = {0};
    data.startCode = SWOP(0xa5a5);
    data.len = SWOP(0x000c);
    data.cnt = SWOP(myData.cnt);
    data.id = SWOP(0x0202);
    data.body1 = SWOP(vel);
    data.body2 = SWOP(w);
    uint16 check = CRC16_CCITT_FALSE(data.units + 2, 10);
    data.checkCode = SWOP(check);
    data.endCode = SWOP(0x5a5a);
    ros_ser.write(data.units, 16);
    myData.cnt++;
    //cout<<"write: ";
    //for(int i=0;i<16;i++)
    //    cout<<setbase(16)<<int(data.units[i])<<" ";
    //cout<<endl;
}

//a5 a5 00 12 00 fd 02 06 ff 01 01 02 03 04 05 06 07 08 f1 f4 5a 5a//设置上报时间


//a5 a5 00 10 00 0b 02 06 3f 00 01 02 03 04 05 06 a7 ba 5a 5a

void AskReportRegularly() {
    UART_TYPE12 data = {0};
    data.startCode = SWOP(0xa5a5);
    data.len = SWOP(0x0012);
    data.cnt = SWOP(myData.cnt);
    data.id = SWOP(0x0206);
    //0x78 11111000
    //data.bitmask0 = 0xf8;
    data.bitmask0 = 0xf9;
    //data.bitmask0 = 0xff;
    data.bitmask1 = 0x01;
    data.TOF = 0x00;
    data.Cur = 0x00;
    data.Signal = 0x00;
    data.IMU = 0x00;
    data.Euler = 0x00;
    data.Encoder = 0x00;
    data.Pose = 0x00;
    data.Time = 0x00;
    uint16 check = CRC16_CCITT_FALSE(data.units + 2, 0x10);
    data.checkCode = SWOP(check);
    data.endCode = SWOP(0x5a5a);
    ros_ser.write(data.units, 22);
    myData.cnt++;
}
