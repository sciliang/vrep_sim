#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP
#include "Driver.hpp"

struct DPYDKZ
{
    ControllerMsg TeleControlData;
};

struct DLXWSZ
{
    short positiveTorque;
    short negativeTorque;
};

struct FSLD
{
    double longitude[30];
    double latitude[30];
};

// message que qt to vrep
typedef struct
{
    __syscall_slong_t mtype;
    ControllerMsg ControllerMsg_;
} QT_msg2Coppeliasim;

class communication
{
private:
    /* data */
public:
    communication(/* args */);
    ~communication();
    char buff_joystick[1024];

    void communication_run(void);
    void Receive_PC_MSG(void);
    void Set2PC_MSG(void);
};

#endif /* COMMUNICATION_HPP */