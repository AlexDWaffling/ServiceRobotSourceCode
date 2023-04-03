#ifndef SERVICE_MOTOR_CONTROLLER_H_
#define SERVICE_MOTOR_CONTROLLER_H_

#include "variant.h"

#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define CCW                            0
#define CW                           1

#define AGV_Ezi_LIMIT_MAX_VELOCITY 500000 // PPS
#define AGV_Ezi_LIMIT_MIN_VELOCITY 0   // PPS

#define pi 3.14

class ServiceMotorController
{
private:
    bool flag_stop;
    bool flag_run_slv1;
    bool flag_run_slv2;
public:
    ServiceMotorController();
    ~ServiceMotorController();
    bool init();
    bool ezi_readEncoder(int32_t &left_value, int32_t &right_value);
    bool ezi_writeVelocity(int32_t left_value, int32_t right_value);
    bool ezi_controlMotor(const float wheel_radius, const float wheel_separation, float *value);
};

#endif // Lib controller mobile robot using Ezi-Plus-E