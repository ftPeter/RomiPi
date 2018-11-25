/*  PID controller inspired by:
     https://github.com/jfstepha/differential-drive/blob/master/scripts/pid_velocity.py

     TODO : Integral term of PID seems broken. Try to tune at some point.
*/
#include <PID_v1.h>
#include "motor.h"

/* methods smoothing instantaneous wheel velocities reported by
   the odometry methods get_instant_left_wheel_vel(), get_instant_right_wheel_vel()
*/

/* arrays for calculating moving window average wheel velocity */
const int _window_size = 10;
float _left_vel_window[_window_size];
float _right_vel_window[_window_size];

float _pid_get_left_average_wheel_velocity() {
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


double left_target_velocity, left_measured_velocity, left_motor_speed;
double right_target_velocity, right_measured_velocity, right_motor_speed;

const double Kp = 300.0;
const double Ki = 500.0;
const double Kd = 10.0;

PID left_pid(&left_measured_velocity, &left_motor_speed, &left_target_velocity, Kp, Ki, Kd, DIRECT);
PID right_pid(&right_measured_velocity, &right_motor_speed, &right_target_velocity, Kp, Ki, Kd, DIRECT);

void pid_setup() {
  left_target_velocity = 0;
  right_target_velocity = 0;

  left_measured_velocity = 0;
  right_measured_velocity = 0;

  //turn the PID on
  left_pid.SetSampleTime(50);
  right_pid.SetSampleTime(50);
  
  left_pid.SetOutputLimits(-300,300);
  right_pid.SetOutputLimits(-300,300);
  
  left_pid.SetMode(AUTOMATIC);  
  right_pid.SetMode(AUTOMATIC);  
}

void doPID() {
  /* PID SETUP */
  left_target_velocity = get_left_wheel_target_velocity();
  right_target_velocity = get_right_wheel_target_velocity();

  // instantaneous velocity
  left_measured_velocity = get_instant_left_wheel_vel();
  right_measured_velocity = get_instant_right_wheel_vel();
  
  // moving window averaged velocity
  //left_measured_velocity = _pid_get_left_average_wheel_velocity();
  //right_measured_velocity = _pid_get_right_average_wheel_velocity();

  /* compute PIDS */
  left_pid.Compute();
  right_pid.Compute();

  /* SET THE MOTORS */
  hw_motors_setspeeds(left_motor_speed, right_motor_speed);
}
