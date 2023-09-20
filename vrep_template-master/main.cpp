/* Includes ------------------------------------------------------------------*/
#include "CoppeliaSim.h"
#include "core_test/car.h"
#include "function.hpp"
#include <sys/msg.h>
#include <iostream>
#include <string>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <thread>
#include <math.h>

QT_msg2Coppeliasim msg_QT2coppSim;

#define MSGQUE_Coppsim2MAIN 510
#define MSGQUE_QT2Coppsim 520

#define VEHICLE_WIDTH 0.2
#define VEHICLE_scale 50

/* Usr defines ---------------------------------------------------------------*/
using namespace std;
/*Object Handle -----------------------------------------------------------*/
_simObjectHandle_Type *Body;
_simObjectHandle_Type *Joint[5];
/*Signal Handle -----------------------------------------------------------*/
_simSignalHandle_Type *EulerAngle[3];    // r、p、y
_simSignalHandle_Type *Position[3];      // position
_simSignalHandle_Type *ActualVel[3];     // velocity
_simSignalHandle_Type *AttitudeAngle[3]; // 姿态角

/*Usr Parameters ----------------------------------------------------------*/
mycar car;
/* Founctions ----------------------------------------------------------------*/
uint32_t getSimTime();

/**
 * @brief This is the main function for user.
 */
void Usr_Main(float tar_left_vel, float tar_right_vel)
{
    // core code
    car.car_spin(tar_left_vel, tar_right_vel);
}

/**
 * @brief User can config simulation client in this function.
 * @note  It will be called before entering the main loop.
 */
void Usr_ConfigSimulation()
{
    // 定义身体和电机
    Body = CoppeliaSim->Add_Object("car", OTHER_OBJECT, {SIM_ORIENTATION | CLIENT_RO, SIM_VELOCITY | CLIENT_RO}); // Readonly
    // 关节
    Joint[lf_wheel] = CoppeliaSim->Add_Object("lf_wheel", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW, SIM_FORCE | CLIENT_RO});
    Joint[lh_wheel] = CoppeliaSim->Add_Object("lh_wheel", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW, SIM_FORCE | CLIENT_RO});
    Joint[rf_wheel] = CoppeliaSim->Add_Object("rf_wheel", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW, SIM_FORCE | CLIENT_RO});
    Joint[rh_wheel] = CoppeliaSim->Add_Object("rh_wheel", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW, SIM_FORCE | CLIENT_RO});

    Joint[servo] = CoppeliaSim->Add_Object("servo", JOINT, {SIM_VELOCITY | CLIENT_RW, SIM_POSITION | CLIENT_RW, SIM_FORCE | CLIENT_RO});

    EulerAngle[yaw] = CoppeliaSim->Add_Object("car.YawAng", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RO});
    EulerAngle[pitch] = CoppeliaSim->Add_Object("car.PitchAng", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RO});
    EulerAngle[roll] = CoppeliaSim->Add_Object("car.RollAng", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RO});

    Position[0] = CoppeliaSim->Add_Object("car.X", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RO});
    Position[1] = CoppeliaSim->Add_Object("car.Y", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RO});
    Position[2] = CoppeliaSim->Add_Object("car.Z", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RO});

    ActualVel[0] = CoppeliaSim->Add_Object("car.VX", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RO});
    ActualVel[1] = CoppeliaSim->Add_Object("car.VY", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RO});
    ActualVel[2] = CoppeliaSim->Add_Object("car.VZ", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RO});

    AttitudeAngle[0] = CoppeliaSim->Add_Object("car.R_roll", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RO});
    AttitudeAngle[1] = CoppeliaSim->Add_Object("car.P_pitch", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RO});
    AttitudeAngle[2] = CoppeliaSim->Add_Object("car.Y_yaw", SIM_FLOAT_SIGNAL, {SIM_SIGNAL_OP | CLIENT_RO});
}

/**
 * @brief These two function will be called for each loop.
 *        User can set their message to send or read from sim enviroment.
 */
void Usr_SendToSimulation()
{

    for (int i = 0; i < 5; i++)
    {
        if (i == servo)
        {
            Joint[i]->obj_Target.angle_f = car.mycar_data.joint_data.out[i];
            return;
        }
        Joint[i]->obj_Target.angVelocity_f = car.mycar_data.joint_data.out[i]; // 实例化数据载体
    }
}

void send2MAIN_msg(void)
{
    CoppSimMSG2Main msg_copSim2main;
    // message que2main
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
        // send 非阻塞，send 的频率就是msgrcv的频率
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 10ms=100Hz
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
        // cout << "diff=" << diff << " s" << endl;

        // data copy and message
        msg_copSim2main.mtype = 1;
        msg_copSim2main.CoppSimmsg2Human.POS_X = car.mycar_data.POS_X;
        msg_copSim2main.CoppSimmsg2Human.POS_Y = car.mycar_data.POS_Y;
        msg_copSim2main.CoppSimmsg2Human.POS_Z = car.mycar_data.POS_Z;

        msg_copSim2main.CoppSimmsg2Human.roll = car.mycar_data.roll;
        msg_copSim2main.CoppSimmsg2Human.pitch = car.mycar_data.pitch;
        msg_copSim2main.CoppSimmsg2Human.yaw = car.mycar_data.yaw;

        msg_copSim2main.CoppSimmsg2Human.roll_new = car.mycar_data.roll_new;
        msg_copSim2main.CoppSimmsg2Human.pitch_new = car.mycar_data.pitch_new;
        msg_copSim2main.CoppSimmsg2Human.yaw_new = car.mycar_data.yaw_new;
        // cout << "\n <------ALL POS ATT------>" << endl
        //      << "XYZ position:"
        //      << "POS_X =" << msg_copSim2main.CoppSimmsg2Human.POS_X << ","
        //      << "POS_Y =" << msg_copSim2main.CoppSimmsg2Human.POS_Y << ","
        //      << "POS_Z =" << msg_copSim2main.CoppSimmsg2Human.POS_Z << endl;

        // cout << "RPY att:"
        //      << "roll =" << msg_copSim2main.CoppSimmsg2Human.roll_new << ","
        //      << "pitch =" << msg_copSim2main.CoppSimmsg2Human.pitch_new << ","
        //      << "yaw =" << msg_copSim2main.CoppSimmsg2Human.yaw_new << endl;

        // send message
        int ret = msgsnd(msgQue_coppSim2MAIN, (void *)&msg_copSim2main, sizeof(msg_copSim2main.CoppSimmsg2Human), IPC_NOWAIT);
        if (ret < 0)
        {
            printf("msgQue_coppSim2MAIN send failed,ret=%d\n", ret);
        }
    }
}

void receiveQt_msg(void)
{
    pthread_mutex_t QT2coppSim_mutex;
    char QT2coppSimBuf[1024];
    // msg que qt to coppeliasim
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

    while (true)
    {
        // cout << "vrep recv QT start,circle" << endl;
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
        // cout << "receiveQt_msg,diff=" << diff << " s" << endl;
        int ret = msgrcv(msgQue_QT2coppSim, (void *)QT2coppSimBuf, QT2CoppeliaSimBufSize, 0, 0);
        if (ret > 0)
        {
            // if (pthread_mutex_lock(&QT2coppSim_mutex) != 0)
            // {
            //     printf("lock error!\n");
            // }
            memcpy(&msg_QT2coppSim, QT2coppSimBuf, QT2CoppeliaSimBufSize);
            // cout << "\n msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity = " << msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity << endl;
            // cout << "msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity = " << msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity << endl;
        }
        else
        {
            printf("[WARNING] msgrcv return :%d.\n", ret);
        }
    }
}

/**
 * @brief 从仿真的小车读数据，数据的拷贝，如果在小车里面加新的传感器，在这个位置赋值
 *
 */
void Usr_ReadFromSimulation()
{
    for (int i = 0; i < 4; i++)
    {
        car.mycar_data.joint_data.curr_pos[i] = Joint[i]->obj_Data.angle_f;
        // car.mycar_data.joint_data.curr_vel[i] = Joint[i]->obj_Data.angVelocity_f;
        car.mycar_data.joint_data.curr_torque[i] = Joint[i]->obj_Data.torque_f;
    }
    car.mycar_data.roll = EulerAngle[roll]->data;
    car.mycar_data.pitch = EulerAngle[pitch]->data;
    car.mycar_data.yaw = EulerAngle[yaw]->data;

    car.mycar_data.POS_X = Position[0]->data;
    car.mycar_data.POS_Y = Position[1]->data;
    car.mycar_data.POS_Z = Position[2]->data;

    car.mycar_data.V_X = ActualVel[0]->data;
    car.mycar_data.V_Y = ActualVel[1]->data;
    car.mycar_data.V_Z = ActualVel[2]->data;

    car.mycar_data.roll_new = AttitudeAngle[0]->data;
    car.mycar_data.pitch_new = AttitudeAngle[1]->data;
    car.mycar_data.yaw_new = AttitudeAngle[2]->data;

    // for (size_t i = 0; i < 4; i++)
    // {
    //     cout << "vel" << i <<:  " <<car.mycar_data.joint_data.curr_vel[i];
    // }

    // cout << "RPY angle:"
    //      << car.mycar_data.roll << ","
    //      << car.mycar_data.pitch << ","
    //      << car.mycar_data.yaw << endl;

    // cout << "XYZ Position:"
    //      << car.mycar_data.POS_X << ","
    //      << car.mycar_data.POS_Y << ","
    //      << car.mycar_data.POS_Z << endl;

    // cout << "XYZ att:"
    //      << car.mycar_data.roll_new << ","
    //      << car.mycar_data.pitch_new << ","
    //      << car.mycar_data.yaw_new << endl;
}

/**
 * @brief It's NOT recommended that user modefies this function.
 *        Plz programm the functions with the prefix "Usr_".
 *         主要是完成和vrep小车的通信，接收底层的信息和向底层发消息！！
 */
int main(int argc, char *argv[])
{
    // 线速度 和主履带转速
    float LEG_VELOCITY_FACTOR = VEHICLE_scale;
    // 角速度 和主履带转速
    float LEG_ANGLE_FACTOR = 30 * VEHICLE_scale * M_PI * VEHICLE_WIDTH / 2 / 180.0;
    float tar_leftvel, tar_rightvel;
    CoppeliaSim_Client *hClient = &CoppeliaSim_Client::getInstance();
    /*
        System Logger tool init.
    */
    std::cout << "[System Logger] Configuring... \n";
    std::cout << "[System Logger] Logger is ready ! \n";
    /*
        Simulation connection init.
    */
    std::cout << "[CoppeliaSim Client] Connecting to server.. \n";
    // vrep的ip地址 端口号 是否选择为同步模式(阻塞模式) 客户端的配置
    while (!hClient->Start("127.0.0.1", 5000, 5, true))
    {
    };
    // vrep的ip地址 端口号 是否选择为非同步模式(非阻塞模式) 客户端的配置
    // while (!hClient->Start("127.0.0.1", 5000, 5, false))
    // {
    // };
    std::cout << "[CoppeliaSim Client] Successfully connected to server, configuring...\n";
    Usr_ConfigSimulation();
    std::cout << "[CoppeliaSim Client] Configure done, simulation is ready ! \n";
    // receive msg from qt 接收由主程序转发过来的遥控数据
    std::thread ReceiveQt(receiveQt_msg);
    // send msg to Main 给主程序发送仿真软件的值
    std::thread Send2Main(send2MAIN_msg);
    while (true)
    {
        if (hClient->Is_Connected())
        {
            hClient->ComWithServer();
        }

        // 计算循环时间
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
        cout << "CoppeliaSim_Client time cost =" << diff << endl;

        // read coppeliasim
        /**
         * 红旗侧为车头
         * 右手边为右侧，左手边为左侧
         * 目前看计算的可能不太对，但是实际的车的表现是对的！！
         */
        Usr_ReadFromSimulation();
        tar_leftvel = LEG_VELOCITY_FACTOR * msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity - LEG_ANGLE_FACTOR * msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity;
        tar_rightvel = LEG_VELOCITY_FACTOR * msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity + LEG_ANGLE_FACTOR * msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity;

        // cout << "22msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity = " << msg_QT2coppSim.ControllerMsg_.vehicleLineVelocity << endl;
        // cout << "22msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity = " << msg_QT2coppSim.ControllerMsg_.vehicleAngularVelocity << endl;
        // cout << "!!LEG_ANGLE_FACTOR = " << LEG_ANGLE_FACTOR << endl;
        cout << "tar_leftvel = " << tar_leftvel << endl;
        cout << "tar_rightvel = " << tar_rightvel << endl;
        // core function
        Usr_Main(tar_leftvel, tar_rightvel);
        // send coppeliasim
        Usr_SendToSimulation();
    };
    ReceiveQt.join();
    Send2Main.join();
}

uint32_t getSimTime()
{
    return 0;
}
/************************* END-OF-FILE SCUT-ROBOTLAB **************************/