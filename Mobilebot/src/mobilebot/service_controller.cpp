#include "../../include/mobilebot/service_motor_controller.h"
#include "EziServoPlusE.h"

IPAddress ip_slave1(192, 168, 0, 2);
IPAddress ip_slave2(192, 168, 0, 7);

EziServoPlusE myServo;

ServiceMotorController::ServiceMotorController()
{
}

ServiceMotorController::~ServiceMotorController()
{
}

bool ServiceMotorController::init()
{
    uint8_t com_status = 0;
    myServo.initServo(ip_slave1, ip_slave2);

    myServo.ServoEnable(ip_slave1, 1, &com_status);
    myServo.ServoEnable(ip_slave2, 1, &com_status);

    myServo.MoveStop(ip_slave1, &com_status);
    myServo.MoveStop(ip_slave2, &com_status);

    myServo.ClearPosition(ip_slave1, &com_status);
    myServo.ClearPosition(ip_slave2, &com_status);

    return true;
}

bool ServiceMotorController::ezi_readEncoder(int32_t &left_value, int32_t &right_value)
{
    uint8_t com_status = 0;
    // Get data
    // left_value = 0;
    // right_value = 0;
    myServo.GetActualPos(ip_slave1, &left_value, &com_status);
    myServo.GetActualPos(ip_slave2, &right_value, &com_status);
    return true;
}

bool ServiceMotorController::ezi_writeVelocity(int32_t left_value, int32_t right_value)
{
    return true;
}

bool ServiceMotorController::ezi_controlMotor(const float wheel_radius, const float wheel_separation, float *value)
{
    bool dxl_comm_result = false;
    uint8_t com_status = 0;

    float wheel_velocity_cmd[2];
    uint32_t wheel_pps_cmd[2];

    float lin_vel = value[LINEAR];
    float ang_vel = value[ANGULAR];

    // float lin_vel = 0.1;
    // float ang_vel = 0.1;

    wheel_velocity_cmd[LEFT] = lin_vel - (ang_vel * wheel_separation / 2);
    wheel_velocity_cmd[RIGHT] = lin_vel + (ang_vel * wheel_separation / 2);

    wheel_pps_cmd[LEFT] = constrain((abs(wheel_velocity_cmd[LEFT]) / (2 * pi * wheel_radius)) * 100000, AGV_Ezi_LIMIT_MIN_VELOCITY, AGV_Ezi_LIMIT_MAX_VELOCITY);
    wheel_pps_cmd[RIGHT] = constrain((abs(wheel_velocity_cmd[RIGHT]) / (2 * pi * wheel_radius)) * 100000, AGV_Ezi_LIMIT_MIN_VELOCITY, AGV_Ezi_LIMIT_MAX_VELOCITY);

    //Debug_Serial.print("Speed_L:     ");
   // Debug_Serial.println(wheel_pps_cmd[LEFT]);
    //Debug_Serial.print("Speed_R:     ");
    //Debug_Serial.println(wheel_pps_cmd[RIGHT]);

    if (wheel_velocity_cmd[LEFT] == 0 && wheel_velocity_cmd[RIGHT] == 0)
    {
        // myServo.MoveVelocity(ip_slave1, 800000, 0, &com_status);
        //     myServo.VelocityOverride(ip_slave1, wheel_pps_cmd[LEFT], &com_status);
        //                 myServo.MoveVelocity(ip_slave2, 1000000, 1, &com_status);
        //     myServo.VelocityOverride(ip_slave2, wheel_pps_cmd[RIGHT], &com_status);
        // myServo.MoveStop(ip_slave1, &com_status);
        // myServo.MoveStop(ip_slave2, &com_status);
        // myservo.ServoEnable(ip_slave1, 0);
        // myservo.ServoEnable(ip_slave2, 0);
    }
    else
    {
        /* Đã sửa từ 400000 -> 100000 */
        if (wheel_velocity_cmd[LEFT] > 0)
        {
            myServo.MoveVelocity(ip_slave1, 100000, 0, &com_status);
            myServo.VelocityOverride(ip_slave1, wheel_pps_cmd[LEFT], &com_status);
        }
        if (wheel_velocity_cmd[LEFT] < 0)
        {
            myServo.MoveVelocity(ip_slave1, 100000, 1, &com_status);
            myServo.VelocityOverride(ip_slave1, wheel_pps_cmd[LEFT], &com_status);
        }
        if (wheel_velocity_cmd[RIGHT] > 0)
        {
            myServo.MoveVelocity(ip_slave2, 100000, 1, &com_status);
            myServo.VelocityOverride(ip_slave2, wheel_pps_cmd[RIGHT], &com_status);
        }
        if (wheel_velocity_cmd[RIGHT] < 0)
        {
            myServo.MoveVelocity(ip_slave2, 100000, 0, &com_status);
            myServo.VelocityOverride(ip_slave2, wheel_pps_cmd[RIGHT], &com_status);
        }
    }
    /*
    else
    {
        if (firt_start_m_left)
        {
            if (wheel_velocity_cmd[LEFT] > 0)
            {
                myServo.MoveVelocity(ip_slave1, 400000, 1, &com_status);
                myServo.VelocityOverride(ip_slave1, wheel_velocity_cmd[LEFT], &com_status);
            }
            if (wheel_velocity_cmd[LEFT] < 0)
            {
                myServo.MoveVelocity(ip_slave1, 400000, 0, &com_status);
                myServo.VelocityOverride(ip_slave1, wheel_pps_cmd[LEFT], &com_status);
            }
            Debug_Serial.println(wheel_pps_cmd[LEFT]);
            firt_start_m_left = false;
        }
        else
        {
            myServo.MoveVelocity(ip_slave1, 400000, 1, &com_status);
            myServo.VelocityOverride(ip_slave1, wheel_pps_cmd[LEFT], &com_status);
        }
        if (firt_start_m_right)
        {
            if (wheel_velocity_cmd[RIGHT] > 0)
            {
                myServo.MoveVelocity(ip_slave2, 400000, 1, &com_status);
                myServo.VelocityOverride(ip_slave2, wheel_pps_cmd[RIGHT], &com_status);
            }
            if (wheel_velocity_cmd[RIGHT] < 0)
            {
                myServo.MoveVelocity(ip_slave2, 400000, 0, &com_status);
                myServo.VelocityOverride(ip_slave2, wheel_pps_cmd[RIGHT], &com_status);
            }
            firt_start_m_right = false;
        }
        else
        {
            myServo.VelocityOverride(ip_slave2, wheel_pps_cmd[RIGHT], &com_status);
        }
        */
    dxl_comm_result = ezi_writeVelocity((int64_t)wheel_velocity_cmd[LEFT], (int64_t)wheel_velocity_cmd[RIGHT]);
    if (dxl_comm_result == false)
        return false;
    return true;
}