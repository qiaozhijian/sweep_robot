//
// Created by qzj on 2020/6/24.
//

#include "command.h"

extern serial::Serial ros_ser;
SerialData myData;

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

void GetSensorStatus(vector<uint8> data){
    size_t totalLen = data.size();
    size_t lenRead = 0;
    uint16_t frameLen = CAT(data[2], data[3]);
    size_t lenTmp = frameLen + 4;
    //处理一次读入多条信息
    while(lenRead<totalLen)
    {
        vector<uint8_t> dataTmp(data.begin() + lenRead, data.begin() + lenRead + lenTmp);
        uint16_t startCode = CAT(dataTmp[0], dataTmp[1]);
        frameLen = CAT(dataTmp[2], dataTmp[3]);
        lenTmp = frameLen + 4;
        uint16_t bodyLen = frameLen - 8;
        uint16_t cntReply = CAT(dataTmp[4], dataTmp[5]);
        uint16_t id = CAT(dataTmp[6], dataTmp[7]);
        vector<uint8_t > body(dataTmp.begin() + 8, dataTmp.begin() + 8 + bodyLen);
        uint16_t endCode = CAT(dataTmp[frameLen+2], dataTmp[frameLen+3]);
        //std::cout << "\nbody: ";
        //for(auto el : body) {
        //    std::cout << int(el) << " ";
        //}
        //std::cout << "\n";
        if(startCode==0xa5a5&&endCode==0x5a5a)
        {
            cout<<"heard "<<setbase(10)<<lenTmp<<" bytes: ";
            for(int i=0;i<lenTmp;i++)
                cout<<setbase(16)<<int(dataTmp[i])<<" ";
            cout<<endl;
            //cout<<setbase(16)<<int(id)<<endl;
            switch (id){
                case 0x8102:
                    if(CAT(body[0], body[1])==0xffff)
                        myData.isAllOn = true;
                    break;
            }
        } else
            cout<<"heard "<<setbase(10)<<lenTmp<<" wrong bytes."<<endl;
        lenRead = lenRead + lenTmp;
    }
}

// a5 a5 00 08 00 c8 01 02 90 18 5a 5a   		//获取传感器使能状态
void AskSensorStatus() {
    UART_TYPE08 data = {0};
    data.startCode = SWOP(0xa5a5);
    data.len = SWOP(0x0008);
    data.cnt = SWOP(myData.cnt);
    data.id = SWOP(0x0102);
    uint16 check = CRC16_CCITT_FALSE(data.units+2,6);
    data.checkCode = SWOP(check);
    data.endCode = SWOP(0x5a5a);
    //cout << hex << check << endl;
    ros_ser.write(data.units,12);
    //cout<<"write: ";
    //for(int i=0;i<12;i++)
    //    cout<<setbase(16)<<int(data.units[i])<<" ";
    //cout<<endl;
    myData.cnt++;
}


//A5 A5 00 0A 00 D0 02 01 FF FF 32 C5 5A 5A	//开所有传感器开关
//A5 A5 00 0A 02 CD 02 01 00 00 81 AA 5A 5A	//关所有传感器开关

void TurnAllSwitch(uint8_t mode)
{
    UART_TYPE0A data = {0};
    data.startCode = SWOP(0xa5a5);
    data.len = SWOP(0x000a);
    data.cnt = SWOP(myData.cnt);
    data.id = SWOP(0x0201);
    if(mode==0)
        data.body = SWOP(0x0000);
    else
        data.body = SWOP(0xffff);

    uint16 check = CRC16_CCITT_FALSE(data.units+2,8);
    data.checkCode = SWOP(check);
    data.endCode = SWOP(0x5a5a);
    ros_ser.write(data.units,14);
    myData.cnt++;
}

void Move(float vel_ms, float w_rads)
{
    int16_t vel = int16_t(vel_ms * 1000.f);
    int16_t w = int16_t(w_rads * 1000.f);
    UART_TYPE0C data = {0};
    data.startCode = SWOP(0xa5a5);
    data.len = SWOP(0x000c);
    data.cnt = SWOP(myData.cnt);
    data.id = SWOP(0x0202);
    data.body1 = SWOP(vel);
    data.body2 = SWOP(w);
    uint16 check = CRC16_CCITT_FALSE(data.units+2,10);
    data.checkCode = SWOP(check);
    data.endCode = SWOP(0x5a5a);
    ros_ser.write(data.units,16);
    myData.cnt++;
    //cout<<"write: ";
    //for(int i=0;i<16;i++)
    //    cout<<setbase(16)<<int(data.units[i])<<" ";
    //cout<<endl;
}
