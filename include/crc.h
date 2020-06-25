#ifndef _CRC_H_
#define _CRC_H_

#include "global.h"

namespace crc
{
    class CRC
    {

    public:
        CRC();//构造函数
        ~CRC();//构造函数
        int cc11;
        void sayHello(void);
        uint8_t sum(uint8_t *buf,uint8_t i,uint8_t number);
        unsigned int CRC16(unsigned char* pchMsg, unsigned int wDataLen);
    };



}
#endif