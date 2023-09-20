#ifndef DRIVER_HPP
#define DRIVER_HPP
#pragma pack(1)

union
{
    float testvalue;
    char buffer[4];
} RobotLift;

union
{
    float testvaluecheck;
    char buffer[4];
} RobotLiftcheck;

union
{
    float testvaluecheck;
    char buffer[4];
} ReceiveCheck;

typedef struct
{
    float vehicleLineVelocity;
    float vehicleAngularVelocity; // 前轮转角
    float Accerlerator;           // 节气门 开的角度
    float rightArmAngleVelocity;
    float rearLeftArmAngleVelocity;
    float brakernum;
    int BCARArmAngle;
} ControllerMsg;

class Driver
{
private:
    struct RobotParameter
    {
        float VelAcc;     // 线加速度
        float AngularAcc; // 角加速度

        double MaxVel;   // 最大线速度
        float MaxAnguar; // 最大角速度

        float MaxVelAcc;    // 最大速度加速度
        float MaxAnguarAcc; // 最大角速度加速度

        float AxisWidth;          // 轴距
        float WheelWidth;         // 轮距
        float MaxFrontWheelAngle; // 前轮最大转角
    };

    RobotParameter RobotParameter_;

public:
    Driver(/* args */);
    ~Driver();

    static ControllerMsg ControllerMsg_;

    void ReceiveRobPDO();
    void RobDriver_run(void);
    int SETrobPDO();
    int SetCtrlMSG_BY_CAN(float _Vel_Velocity, float _Angle_Velocity);
    int SETrobVel(float _Vel_Velocity, float MAX_Vel, float _Angle_Velocity, float MAX_Angle);
    // 控制量
    // struct Control_MSG
    // {
    //     float LinearVel;
    //     float AngularVel;
    // };
    // Control_MSG Control_MSG_;
    double CacuTracezxAngle(double LinearVelocity, double AngleVelocity);
};
void V_W_CanSend(uint8_t CAN_ID, uint8_t byte_1, uint8_t byte_2, uint8_t byte_3, uint8_t *byte_45, uint8_t *byte_67);
void VELandANGsend(float Velocity, float Angular, uint8_t HeaderID, uint8_t SEND_ID);
#endif /* DRIVER */