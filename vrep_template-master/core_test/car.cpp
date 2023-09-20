#include "car.h"
void mycar::car_spin(float targevelLeft, float targevelright)
{
        // global static
        static uint8_t dir = 1;
        static uint8_t cnt = 0;

        // enum servo ,position control
        mycar_data.joint_data.tar_pos[servo] = 0 * 3.14 / 180 * dir;
        if (cnt >= 10)
        {
                cnt = 0;
                dir = !dir;
        }
        cnt++;

        // enum servo ,position control
        /**send value map velocity
         * so value=50 * velocity
         * 40 --- 0.81m/s 40/0.81=49.382
         * 30 --- 0.64m/s 30/0.64=46.875
         * 20 --- 0.48m/s 20/0.48=41.666
         * 10 --- 0.23m/s 10/0.23=43.478
         * */
        // targevelright = 50;
        // targevelLeft = 50;
        for (size_t i = 0; i < 2; i++)
                mycar_data.joint_data.tar_vel[i] = -targevelLeft;
        for (size_t i = 2; i < 4; i++)
                mycar_data.joint_data.tar_vel[i] = -targevelright;

        // data output
        for (size_t i = 0; i < 5; i++)
        {
                if (i == servo)
                {
                        mycar_data.joint_data.out[i] = mycar_data.joint_data.tar_pos[i];
                        return;
                }
                mycar_data.joint_data.out[i] = mycar_data.joint_data.tar_vel[i];
        }
}

mycar::mycar(/* args */) {}
mycar::~mycar() {}
