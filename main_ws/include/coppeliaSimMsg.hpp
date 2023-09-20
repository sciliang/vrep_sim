#pragma once
#include <iostream>
#include "car.h"
// 和main工程间的消息队列
typedef struct
{
    __syscall_slong_t mtype;
    CarDataTypedef CoppSimmsg2Human;
} CoppSimMSG2Main;

class CoppeliaSim
{
private:
    /* data */
public:
    CoppeliaSim(/* args */);
    ~CoppeliaSim();
    int CoppeliaSim_run(void);
    int InvertSimPOS2GPS(double SimX, double SimY, float SimZ, double invertPOS[2]);
    int InvertSimAtt2ActualAtt(float SimAttX, float SimAttY, float SimAttZ, float invertATT[3]);
    int SendControlMsg_run(void);
};
