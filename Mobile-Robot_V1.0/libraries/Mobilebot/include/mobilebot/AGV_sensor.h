/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho, Gilbert */
/* Maintainer: Indruino */

#ifndef AGV_SENSOR_H_
#define AGV_SENSOR_H_

#include <IMU.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

// #if defined NOETIC_SUPPORT
//   #include <sensor_msgs/BatteryStateNoetic.h>
// #else
//   #include <sensor_msgs/BatteryState.h>
// #endif

#include "OLLO.h"

#define ACCEL_FACTOR                      0.000598550415   // (ADC_Value / Scale) * 9.80665            => Range : +- 2[g]
                                                           //                                             Scale : +- 16384
#define GYRO_FACTOR                       0.0010642        // (ADC_Value/Scale) * (pi/180)             => Range : +- 2000[deg/s]
                                                           //                                             Scale : +- 16.4[deg/s]

#define MAG_FACTOR                        15e-8

// #define DEBUG_SERIAL  Serial6

typedef struct LED_PIN_ARRAY
{
  int front_left;
  int front_right;
  int back_left;
  int back_right;
}LedPinArray;
 
typedef struct SONAR_PIN
{
  int trig;
  int echo;
}SonarPin;

typedef struct Ben_PIN
{
  int pwm;
  int dir;
}BenPin;

typedef struct INPUT_PIN
{
  int in1;
  int in2;
  int in3;
  int in4;
  int in5;
  int in6;
  int in7;
  int in8;
}InputPin;

typedef struct OUTPUT_PIN
{
  int out1;
  int out2;
  int out3;
  int out4;
  int out5;
  int out6;
  int out7;
  int out8;
}OutputPin;

class AGVSensor
{
 public:
  AGVSensor();
  ~AGVSensor();

  bool init(void);

  // IMU
  void initIMU(void);
  sensor_msgs::Imu getIMU(void);
  void updateIMU(void);
  void calibrationGyro(void);

  float* getOrientation(void);
  sensor_msgs::MagneticField getMag(void);

  // Battery
  float checkVoltage(void);

  // Button
  uint8_t checkPushButton(void);

  // Sound
  void melody(uint16_t* note, uint8_t note_num, uint8_t* durations);
  void makeSound(uint8_t index);  

  // Bumper
  void initBumper(void);
  uint8_t checkPushBumper(void);

  // Cliff sensor
  void initIR(void);
  float getIRsensorData(void);

  // setup io
  void initIO(void);
  void benControl(uint8_t pwm,uint8_t dir);
  void outputControl(uint8_t pin,uint8_t status);
  byte readinput();

  // Sonar sensor
  //void initSonar(void);
  //void updateSonar(uint32_t t);
  //float getSonarData(void);

  // Illumination sensor
  float getIlluminationData(void);

  // led pattern
  //void initLED(void);
  //void setLedPattern(double linear_vel, double angular_vel);
 private:
  sensor_msgs::Imu           imu_msg_;
  // #if defined NOETIC_SUPPORT
  //   sensor_msgs::BatteryStateNoetic  battery_state_msg_;
  // #else
  //   sensor_msgs::BatteryState  battery_state_msg_;
  // #endif
  sensor_msgs::MagneticField mag_msg_;

  cIMU imu_;
  OLLO ollo_;

  LedPinArray led_pin_array_;
  SonarPin sonar_pin_;

  InputPin input_pin_;
  OutputPin output_pin_;
  BenPin ben_pin_;

  float sonar_data_;
};

#endif // AGV_SENSOR_H_
