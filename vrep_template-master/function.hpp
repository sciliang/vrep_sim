#pragma once
#include <iostream>
#include "car.h"
#pragma pack(1)
// 和main工程间的消息队列
typedef struct
{
    __syscall_slong_t mtype;
    CarDataTypedef CoppSimmsg2Human;
} CoppSimMSG2Main;

// message que qt to vrep
typedef struct
{
    float vehicleLineVelocity;
    float vehicleAngularVelocity; 
    float Accerlerator;           
    float rightArmAngleVelocity;
    float rearLeftArmAngleVelocity;
    float brakernum;
    int BCARArmAngle;
} ControllerMsg;

typedef struct
{
    __syscall_slong_t mtype;
    ControllerMsg ControllerMsg_;
} QT_msg2Coppeliasim;

void receiveQt_msg(void);
void send2MAIN_msg(void);