#ifndef GLOBAL_HPP
#define GLOBAL_HPP
#include <iostream>
#include "communication.hpp"
#include <mutex>
// 全局变量

class global
{
private:
    /* data */
public:
    global(/* args */);
    ~global();
    typedef struct
    {
        double longitude[300];
        double latitude[300];
    } LatANDLon;
    // static std::mutex GPSPOS_flag_mtx;
    static double longitude;
    static double latitude;
    static float heading;
};
struct tm *getsystime(void);
void signal_handler(int signum);

#endif