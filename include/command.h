//
// Created by qzj on 2020/6/24.
//

#ifndef SRC_COMMAND_H
#define SRC_COMMAND_H
#include "global.h"

class SerialData{
public:
    int cnt=0;
    bool isAllOn = false;

private:
    int a,b,c;

};


void AskSensorStatus();
void GetSensorStatus(vector<uint8> data);
void TurnAllSwitch(uint8_t mode);

#endif //SRC_COMMAND_H
