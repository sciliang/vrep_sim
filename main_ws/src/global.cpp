#include <iostream>
#include <glog/logging.h>
#include <signal.h>
#include "global.hpp"
#include <sys/signal.h>
#include <stdio.h>

// std::mutex global::GPSPOS_flag_mtx;
double global::longitude = 0;
double global::latitude = 0;
float global::heading = 0;

FILE *fp_RoadPointSave;
global::global(/* args */)
{
}

global::~global()
{
}


struct tm *getsystime(void)
{
    time_t t;
    struct tm *lt;
    time(&t);           // 获取Unix时间戳。
    lt = localtime(&t); // 转为时间结构。
    //	printf ( "%d/%d/%d %d:%d:%d\n",
    //			lt->tm_year+1900, lt->tm_mon, lt->tm_mday, lt->tm_hour, lt->tm_min, lt->tm_sec);//输出结果
    return lt;
}
