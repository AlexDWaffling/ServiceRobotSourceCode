#ifndef MOBILEBOT_SERVICE_H_
#define MOBILEBOT_SERVICE_H_

#define WHEEL_RADIUS                     0.0725           // meter
#define WHEEL_SEPARATION                 0.433           // meter 
#define TURNING_RADIUS                   0.2185          // meter 
#define ROBOT_RADIUS                     0.275           // meter 
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw

#define MAX_LINEAR_VELOCITY              2.0           // m/s  
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

#define MIN_LINEAR_VELOCITY              -MAX_LINEAR_VELOCITY
#define MIN_ANGULAR_VELOCITY             -MAX_ANGULAR_VELOCITY

#endif // MOBILEBOT_SERVICE_H_
