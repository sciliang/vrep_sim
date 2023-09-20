#include <iostream>
#include <glog/logging.h>
#include <math.h>

// 去掉误差过大的值
double *Filter(double filterlat, double filterlon, double FILTER_A)
{
    static double OldLatitude, OldLongitude;
    double AfterfiltrPos[2]; // 0是经度，1是纬度

    // 经度过滤
    if (fabs((filterlon - OldLongitude) > FILTER_A))
    {
        AfterfiltrPos[0] = OldLongitude;
    }
    else
    {
        AfterfiltrPos[0] = filterlon;
    }

    // 纬度过滤
    if (fabs((filterlat - OldLatitude) > FILTER_A))
    {
        AfterfiltrPos[1] = OldLatitude;
    }
    else
    {
        AfterfiltrPos[1] = filterlat;
    }

    OldLatitude = filterlat;
    OldLongitude = filterlon;
    return AfterfiltrPos;
}

