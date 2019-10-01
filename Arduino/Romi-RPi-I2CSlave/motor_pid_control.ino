/*  PID controller inspired by:
     https://github.com/jfstepha/differential-drive/blob/master/scripts/pid_velocity.py

     TODO : Integral term of PID seems broken. Try to tune at some point.
*/

#include "motor.h"

/* methods smoothing instantaneous wheel velocities reported by
   the odometry methods get_instant_left_wheel_vel(), get_instant_right_wheel_vel()
*/

/* arrays for calculating moving window average wheel velocity */
const int _window_size = 10;
float _left_vel_window[_window_size];
float _right_vel_window[_window_size];



float _pid_get_left_average_wheel_velocity() {
  // WARNING: _pid_get_left_average_wheel_velocity is not tested 
  //           and probably doesn't work
  
  // average instantaneous left wheel velocity
  // in meters per second
  float average = 0;
  static int i;
  _left_vel_window[i++] = get_instant_left_wheel_vel();
  if ( i >= _window_size ) {
    i = 0;
  }
  for (int j = 0; j < _window_size; j++) {
    average += _left_vel_window[i];
  }
  average /= _window_size;
  return average;
}

float _pid_get_right_average_wheel_velocity() {
  // WARNING: _pid_get_left_average_wheel_velocity is not tested 
  //           and probably doesn't work  
  // average instantaneous right wheel velocity
  // in meters per second
  float average = 0;
  static int i;
  _right_vel_window[i++] = get_instant_right_wheel_vel();
  if ( i >= _window_size ) {
    i = 0;
  }
  for (int j = 0; j < _window_size; j++) {
    average += _right_vel_window[i];
  }
  average /= _window_size;
  return average;
}

/*  setMotorSpeeds

    constraint the motor speed within the motor power limit
    then set the speeds
*/
void _pid_setMotorSpeeds(int16_t left_motor, int16_t right_motor,
                         float left_error, float right_error,
                         float *left_integral, float *right_integral,
                         float duration_s ) {
  /* CONSTRAIN MOTOR SETTING TO THE MOTOR POWER LIMITS */
  if ( left_motor > MOTOR_MAX ) {
    left_motor = MOTOR_MAX;
    *left_integral  = *left_integral - (left_error * duration_s);
  } else if ( left_motor < MOTOR_MIN ) {
    left_motor = MOTOR_MIN;
    *left_integral  = *left_integral - (left_error * duration_s);
  }

  if ( right_motor > MOTOR_MAX ) {
    right_motor = MOTOR_MAX;
    *right_integral  = *right_integral - (right_error * duration_s);
  } else if ( right_motor < MOTOR_MIN ) {
    right_motor = MOTOR_MIN;
    *right_integral  = *right_integral - (right_error * duration_s);
  }

  /* REMOVE THIS TO FOR ACTIVE BRAKING */
  if ( get_left_wheel_target_velocity() == 0.0 ) {
    left_motor = 0;
  }
  if ( get_right_wheel_target_velocity() == 0.0 ) {
    right_motor = 0;
  }

  /* SET THE MOTORS */
  hw_motors_setspeeds(left_motor, right_motor);
}

void doPID() {
  /* PID AND MOTOR CONSTANTS
   * note: these constants seem to work,
   * but may need to change depending on
   * physical robot configuration
   * 
   * warning: the derivative term is untested.
   */
  const float Kp = 2000.0;
  const float Ki = 7000.0;
  const float Kd = 0.0;

  /* STATIC VARIABLES */
  static unsigned long prev_time_ms;
  static float left_prev_error, right_prev_error;
  static float left_integral, right_integral;

  /* TIME DURATION SINCE LAST UPDATE */
  unsigned long current_time_ms = millis();
  long duration_ms = current_time_ms - prev_time_ms;
  float duration_s = float(duration_ms) / 1000.0;

  /* CALCULATE ERROR : error = target - measured */
  // TODO change this to use _pid_get_left_average_wheel_velocity, _pid_get_right_average_wheel_velocity
  float left_error  = get_left_wheel_target_velocity() - get_instant_left_wheel_vel();
  float right_error = get_right_wheel_target_velocity() - get_instant_right_wheel_vel();
  /*
   * WARNING, the averaging methods are known to break the code and become unstable.
  float left_error  = get_left_wheel_target_velocity() - _pid_get_left_average_wheel_velocity();
  float right_error = get_right_wheel_target_velocity() - _pid_get_right_average_wheel_velocity();
  */

#ifdef DBG_SRL
  /* this triggers once is the motors are quickly shut off */
  if ( abs(left_error) > 0.1 or abs(right_error) > 0.1) {
    Serial.print("MOTOR ERROR!!!");
    Serial.print(left_error);
    Serial.print(", ");
    Serial.print(right_error);
  }
#endif

  /* PID CALCULATIONS*/
  left_integral  = left_integral  + (left_error * duration_s);
  right_integral = right_integral + (right_error * duration_s);
  float left_derivative  = (left_error - left_prev_error) / duration_s;
  float right_derivative = (right_error - right_prev_error) / duration_s;

  int16_t left_motor  = int16_t(Kp * left_error  + Ki * left_integral  + Kd * left_derivative);
  int16_t right_motor = int16_t(Kp * right_error + Ki * right_integral + Kd * right_derivative);

  _pid_setMotorSpeeds(left_motor, right_motor,
                      left_error, right_error,
                      &left_integral, &right_integral,
                      duration_s);

  // update previous values
  prev_time_ms = current_time_ms;
  left_prev_error  = left_error;
  right_prev_error = right_error;
}
