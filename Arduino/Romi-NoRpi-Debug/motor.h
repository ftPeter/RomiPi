
/*  motor.h

    function prototypes and constants related to the motor system.

    November 24, 2018
*/
#ifndef _MOTOR_H
#define _MOTOR_H

/* ROMI HARDWARE CONSTANTS */
//the distance between two wheels in meters (14 cm)
const float WHEEL_SEPERATION_DIST_M = 0.14;
// wheel diameter in meters (7 cm)
const float WHEEL_DIAM_M = 0.07;
// calculate the distance per tick
const float TICKS_PER_ROTATION = 1440.0;
const float METERS_PER_TICK = (PI * WHEEL_DIAM_M) / TICKS_PER_ROTATION;


/* MOTOR CONSTANTS */
const int  MOTOR_MAX = 300, MOTOR_MIN = -300;
const int  FLIP_MOTOR_DIR = -1;
const bool SWAP_MOTORS = false;


/* ENCODER CONSTANTS
*/
const int16_t MAX_INT16 = 32767;
const int16_t MIN_INT16 = -32768;
const int16_t ENCODER_WIN_HIGH = MAX_INT16 * 0.7;
const int16_t ENCODER_WIN_LOW  = MIN_INT16 * 0.7;

const int FLIP_ENCODER_DIR = -1;
const bool SWAP_ENCODERS = true;


/* MOTOR_PID PROTOTYPES */
void doPID();


/* MOTOR_HARDWARE PROTOTYPES */
int debug_get_left_motor_power();
int debug_get_right_motor_power();
void hw_motors_setspeeds(int left, int right);
int hw_getencoder_left();
int hw_getencoder_right();


/* MOTOR_CMD_TWIST PROTOTYPES */
float get_left_wheel_target_velocity();
float get_right_wheel_target_velocity();
void set_twist_target(float linear_m_s, float angle_rad_s);

/*!
     \brief normalize

     Normalizes the angle to be -M_PI circle to +M_PI circle
     It takes and returns radians.

*/
double normalize_angle(double angle)
{
  double return_a = fmod(fmod(angle, 2.0*PI) + 2.0*PI, 2.0*PI);
  if (return_a > PI)
    return_a -= 2.0 * PI;
  return return_a;
}

#endif //_MOTOR_H
