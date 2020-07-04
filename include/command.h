//
// Created by qzj on 2020/6/24.
//

#ifndef SRC_COMMAND_H
#define SRC_COMMAND_H
#include "global.h"
#include <serial/serial.h>

extern serial::Serial ros_ser;

void Move(float vel_ms, float w_rads);
void AskSensorStatus();
void TurnAllSwitch(uint8_t mode);
void AskReportRegularly();
void HandleUART(vector<uint8> data);
void InfoReceive(uint16_t id);
#endif //SRC_COMMAND_H
