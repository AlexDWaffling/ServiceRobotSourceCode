#include "mobilebot_config.h"

void setup()
{
#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.begin(9600);
#endif
  motor_driver.init();

  nh.initNode();

  nh.subscribe(cmd_vel_sub);
  nh.advertise(sensor_state_pub);
}
bool flag = true;
void loop()
{
  uint32_t t = millis();
  if ((t - tTime[0]) >= (1000 / 30))
  {
    updateGoalVelocity();
    if ((t - tTime[6]) > 1000)
    {
      motor_driver.ezi_controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, zero_velocity);
    }
    else
    {
      motor_driver.ezi_controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
    }
    tTime[0] = t;
  }

  if ((t - tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishSensorStateMsg();
    tTime[2] = t;
  }
  nh.spinOnce();
}

/*******************************************************************************
   Callback function for cmd_vel msg
 *******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR] = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR] = constrain(goal_velocity_from_cmd[LINEAR], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = millis();
}
void updateGoalVelocity(void)
{
  goal_velocity[LINEAR] = goal_velocity_from_cmd[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
}

/*******************************************************************************
 * Update motor information
 *******************************************************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};

  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index] = 0;
      last_rad[index] = 0.0;

      last_velocity[index] = 0.0;
    }

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT] = current_tick;
  last_rad[LEFT] += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT] = current_tick;
  last_rad[RIGHT] += TICK2RAD * (double)last_diff_tick[RIGHT];
}

/*******************************************************************************
 * Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
 *******************************************************************************/
void publishSensorStateMsg(void)
{
  bool dxl_comm_result = false;

  // sensor_state_msg.header.stamp = rosNow();
  //  sensor_state_msg.battery = sensors.checkVoltage();

  dxl_comm_result = motor_driver.ezi_readEncoder(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);

  if (dxl_comm_result == true)
    updateMotorInfo(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);
  else
    return;

  // sensor_state_msg.bumper = sensors.checkPushBumper();

  // sensor_state_msg.cliff = sensors.getIRsensorData();

  // TODO
  // sensor_state_msg.sonar = sensors.getSonarData();

  // sensor_state_msg.illumination = sensors.getIlluminationData();

  // sensor_state_msg.button = sensors.checkPushButton();

  // sensor_state_msg.torque = motor_driver.getTorque();

  sensor_state_pub.publish(&sensor_state_msg);
}
