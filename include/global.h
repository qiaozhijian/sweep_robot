//
// Created by qzj on 2020/6/24.
//

#ifndef SRC_GLOBAL_H
#define SRC_GLOBAL_H

#include "ros/ros.h"
#include <serial/serial.h>
#include "crc.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int32.h"
#include <typeinfo>
#include<time.h>

using namespace std;

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;

typedef union{
    uint16 twoByte;
    uint8  byteArr[2];
}SHORT_CHARS;

typedef union {
    struct {
        unsigned short startCode;
        unsigned short len;
        unsigned short cnt;
        unsigned short id;
        unsigned short checkCode;
        unsigned short endCode;
    };
    unsigned char units[12];
}UART_TYPE08;
typedef union {
    struct {
        unsigned short startCode;
        unsigned short len;
        unsigned short cnt;
        unsigned short id;
        unsigned short body;
        unsigned short checkCode;
        unsigned short endCode;
    };
    unsigned char units[14];
}UART_TYPE0A;
typedef union {
    struct {
        unsigned short startCode;
        unsigned short len;
        unsigned short cnt;
        unsigned short id;
        unsigned char body1;
        unsigned short body2;
        unsigned short checkCode;
        unsigned short endCode;
    };
    unsigned char units[15];
}UART_TYPE0B;
typedef union {
    struct {
        unsigned short startCode;
        unsigned short len;
        unsigned short cnt;
        unsigned short id;
        unsigned short body1;
        unsigned short body2;
        unsigned short checkCode;
        unsigned short endCode;
    };
    unsigned char units[16];
}UART_TYPE0C;

typedef union {
    struct {
        unsigned short startCode;
        unsigned short len;
        unsigned short cnt;
        unsigned short id;
        unsigned char bitmask0;
        unsigned char bitmask1;
        unsigned char TOF;
        unsigned char Cur;
        unsigned char Signal;
        unsigned char IMU;
        unsigned char Euler;
        unsigned char Encoder;
        unsigned char Pose;
        unsigned char Time;
        unsigned short checkCode;
        unsigned short endCode;
    };
    unsigned char units[22];
}UART_TYPE12;

namespace global
{
    typedef struct {
        int global_counter;
    }GLOBAL
    ;
}

#define SWOP(n) (((n & 0x00FF) << 8 ) | ((n & 0xFF00) >> 8))
#define CAT(a,b) (((a & 0x00FF) << 8 ) | (b & 0x00FF))
#define CAT32(a,b,c,d) (((a & 0x000000FF) << 24) | ((b & 0x000000FF) << 16) | ((b & 0x000000FF) << 8) | ((b & 0x000000FF) << 0))
#define foo(arr) (sizeof(arr)/sizeof(arr[0]))


#define BIT_0(n)  (((n) & (1 << 0))!=0)
#define BIT_1(n)  (((n) & (1 << 1))!=0)
#define BIT_2(n)  (((n) & (1 << 2))!=0)
#define BIT_3(n)  (((n) & (1 << 3))!=0)
#define BIT_4(n)  (((n) & (1 << 4))!=0)
#define BIT_5(n)  (((n) & (1 << 5))!=0)
#define BIT_6(n)  (((n) & (1 << 6))!=0)
#define BIT_7(n)  (((n) & (1 << 7))!=0)



#endif //SRC_GLOBAL_H
