#include <cstring>
#include <iostream>
#include "coppeliaSimMsg.hpp"
#include <sys/msg.h>
#include <sys/time.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <iomanip>
#include "global.hpp"
#include <unistd.h>
using namespace std;
#define MSGQUE_Coppsim2MAIN 510
#define MSGQUE_QT2Coppsim 520
#define RADIANS_TO_DEGREES 57.29578
QT_msg2Coppeliasim msg_QT2coppSim;

CoppeliaSim::CoppeliaSim(/* args */)
{
}

CoppeliaSim::~CoppeliaSim()
{
}

/**
 * para1：roll：
 * para2：pitch：
 * para3：yaw：左半圈(0~180)，右半圈(-180~0 )
 */
int CoppeliaSim::InvertSimAtt2ActualAtt(float SimAttX, float SimAttY, float SimAttZ, float invertATT[3])
{
    SimAttX = SimAttX * RADIANS_TO_DEGREES;
    SimAttY = SimAttY * RADIANS_TO_DEGREES;
    if (SimAttZ >= 0)
    {
        SimAttZ = 360 - (SimAttZ * RADIANS_TO_DEGREES);
    }
    else
    {
        SimAttZ = -1 * SimAttZ * RADIANS_TO_DEGREES;
    }
    invertATT[0] = SimAttX;
    invertATT[1] = SimAttY;
    invertATT[2] = SimAttZ;
    // LOG(INFO) << "After att inver:invertATT[0]" << invertATT[0] << ","
    //           << "invertATT[1]" << invertATT[1] << ","
    //           << "invertATT[2]" << invertATT[2] << endl;

    return true;
}



/**
 *  组合导航系统：E-N-U，Vrep_GPS:X-Y-Z
 *  将经纬度映射到单位的花坛坐标附近：纬度：41.76136106度，经度：123.4409071度，海拔高度：77.56米
 *  invertPOS[0] 纬度
 *  invertPOS[1] 经度
 */
int CoppeliaSim::InvertSimPOS2GPS(double SimX, double SimY, float SimZ, double invertPOS[2])
{
    // double ReferZero_longitude = 123.4409071, ReferZero_latitude = 41.76136106;
    double ReferZero_longitude = 123.44214967, ReferZero_latitude = 41.76121699;
    float R_EARTH = 6378393.0; // 地球半径 6378393.0  6371
    // 纬度
    invertPOS[0] = (SimY / ((2 * M_PI * R_EARTH) / 360)) + ReferZero_latitude;
    // 经度
    invertPOS[1] = (SimX / (fabs((2 * M_PI * R_EARTH * cos(invertPOS[0] / RADIANS_TO_DEGREES)) / 360))) + ReferZero_longitude;
    // LOG(INFO) << "SimX,Y transfer Lat and Lon:"
    //           << " invertPOS[0],latitude = " << setprecision(12) << invertPOS[0]
    //           << " invertPOS[1],longitude =" << setprecision(12) << invertPOS[1] << endl;
    return true;
}



int CoppeliaSim::SendControlMsg_run(void)
{
    //---- 消息队列
    int msgQue_QT2coppSim = -1;
    memset(&msg_QT2coppSim, 0, sizeof(msg_QT2coppSim));
    int QT2CoppeliaSimBufSize = sizeof(msg_QT2coppSim);
    msgQue_QT2coppSim = msgget((key_t)MSGQUE_QT2Coppsim, 0666 | IPC_CREAT);
    while (msgQue_QT2coppSim < 0)
    {
        printf("[init message]:create msgque error:%d! try to ReInit...\n",
               msgQue_QT2coppSim);
        msgQue_QT2coppSim = msgget((key_t)MSGQUE_QT2Coppsim, 0666 | IPC_CREAT);
        sleep(1);
    }
    //-----//
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10ms=100Hz
        // caculate cost time
        namespace sc = std::chrono;
        static sc::duration<double> start_time = sc::system_clock::now().time_since_epoch();
        sc::duration<double> now_time = sc::system_clock::now().time_since_epoch();
        double fps = 0;
        double diff = (now_time - start_time).count();
        if (diff > 0)
        {
            fps = 1 / diff;
            start_time = now_time;
        }

        // 赋值速度就好了（你需要做的就是给这个结构体的两个参数赋值就好了！）
        msg_QT2coppSim.mtype = 1;
        msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity = 10;
        msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity = 1;
        // cout << "SendControlMsg_run time cost =" << diff << endl;

        // send message：和vrep进程进行通讯
        int ret = msgsnd(msgQue_QT2coppSim, (void *)&msg_QT2coppSim, sizeof(msg_QT2coppSim.ControllerMsg_), IPC_NOWAIT);
        if (ret < 0)
        {
            // printf("msgQue_coppSim2MAIN send failed,ret=%d\n", ret);
        }
    }
    cout << "SendControlMsg start!!" << endl;
}

/**
 * 接收vrep信息的主体线程
 * 主要为接收GPS、Velocity、IMU
 * 并对上述的信息做了相应的转换
 */
int CoppeliaSim::CoppeliaSim_run(void)
{
    global global_coppeliaSim;
    cout << "Coppelisam_run start .." << endl;
    CoppSimMSG2Main msg_copSim2main;
    char CoppeliasimBuf[1024];
    pthread_mutex_t CoppeliaSim2_mutex;
    double InvertSimPos[2]; // 纬度、经度
    float InvertSimATT[3];  // roll、pitch、yaw

    // msg que to main
    int msgQue_coppSim2MAIN = -1;
    memset(&msg_copSim2main, 0, sizeof(msg_copSim2main));
    int CoppSim2BufSize = sizeof(msg_copSim2main);
    msgQue_coppSim2MAIN = msgget((key_t)MSGQUE_Coppsim2MAIN, 0666 | IPC_CREAT);
    while (msgQue_coppSim2MAIN < 0)
    {
        printf("[init message]:create msgque error:%d! try to ReInit...\n",
               MSGQUE_Coppsim2MAIN);
        msgQue_coppSim2MAIN = msgget((key_t)MSGQUE_Coppsim2MAIN, 0666 | IPC_CREAT);
        sleep(1);
    }

    while (true)
    {
        // cout << "Coppelisam_run start, Circular " << endl;
        // caculate the time cost
        namespace sc = std::chrono;
        static sc::duration<double> start_time = sc::system_clock::now().time_since_epoch();
        sc::duration<double> now_time = sc::system_clock::now().time_since_epoch();
        double fps = 0;
        double diff = (now_time - start_time).count();
        if (diff > 0)
        {
            fps = 1 / diff;
            start_time = now_time;
        }
        // cout << "copSim2main , diff=" << diff << " s" << endl;
        // send(非阻塞)，send 的频率就是msgrcv(阻塞式)频率 ,msgrcv取决于send
        int ret = msgrcv(msgQue_coppSim2MAIN, (void *)CoppeliasimBuf, CoppSim2BufSize, 0, 0);
        if (ret > 0)
        {
            // if (pthread_mutex_lock(&CoppeliaSim2_mutex) != 0)
            // {
            //     printf("lock error!\n");
            // }
            memcpy(&msg_copSim2main, CoppeliasimBuf, CoppSim2BufSize);
            // cout << "before invert:msg_copSim2main.CoppSimmsg2Human.POS_X = " << setprecision(12) << msg_copSim2main.CoppSimmsg2Human.POS_X << endl;
            // cout << "before invert:msg_copSim2main.CoppSimmsg2Human.POS_Y = " << setprecision(12) << msg_copSim2main.CoppSimmsg2Human.POS_Y << endl;
            // cout << "before invert:msg_copSim2main.CoppSimmsg2Human.POS_Z = " << setprecision(12) << msg_copSim2main.CoppSimmsg2Human.POS_Z << endl;

            /**Sim 给的是float，但是程序里面是double，所以float会升级，
             * 精度会受到影响，目前采用的是强制转换！，后续考虑精确转换！*/
            InvertSimPOS2GPS(msg_copSim2main.CoppSimmsg2Human.POS_X, msg_copSim2main.CoppSimmsg2Human.POS_Y, msg_copSim2main.CoppSimmsg2Human.POS_Z, InvertSimPos);
            // cout << "After invert: latitude = " << setprecision(12) << InvertSimPos[0] << endl;
            // cout << "After invert: longitude = " << setprecision(12) << InvertSimPos[1] << endl;

            // cout << "After invert:纬度 latitude = " << InvertSimPos[0] << endl;
            // cout << "After invert:经度 longitude = " << InvertSimPos[1] << endl;

            // cout << "before invert:msg_copSim2main.CoppSimmsg2Human.roll_new = " << setprecision(6) << msg_copSim2main.CoppSimmsg2Human.roll_new << endl;
            // cout << "before invert:msg_copSim2main.CoppSimmsg2Human.pitch_new  = " << setprecision(6) << msg_copSim2main.CoppSimmsg2Human.pitch_new << endl;
            // cout << "before invert:msg_copSim2main.CoppSimmsg2Human.yaw_new = " << setprecision(6) << msg_copSim2main.CoppSimmsg2Human.yaw_new << endl;

            InvertSimAtt2ActualAtt(msg_copSim2main.CoppSimmsg2Human.roll_new, msg_copSim2main.CoppSimmsg2Human.pitch_new, msg_copSim2main.CoppSimmsg2Human.yaw_new, InvertSimATT);
            // cout << "After invert:roll_new = " << setprecision(6) << InvertSimATT[0] << endl;
            // cout << "After invert:pitch_new  = " << setprecision(6) << InvertSimATT[1] << endl;
            // cout << "After invert:yaw_new = " << setprecision(6) << InvertSimATT[2] << endl;
            global_coppeliaSim.heading = InvertSimATT[2];
            global_coppeliaSim.latitude = InvertSimPos[0];
            global_coppeliaSim.longitude = InvertSimPos[1];
            // cout << "After invert:roll_new = " << setprecision(6) << InvertSimATT[0] << endl;
            // cout << "After invert:pitch_new  = " << setprecision(6) << InvertSimATT[1] << endl;
            // cout << "After invert:yaw_new = " << setprecision(6) << InvertSimATT[2] << endl;
        }
        else
        {
            cout << "[WARNING] msgrcv return ret = " << ret << endl;
        }
    }
    return true;
}
