
/* motor_cmd_twist
 * 
 * methods for handling twist commands and converting them 
 *  to target motor velocities
 * 
 * November 24, 2018
 */

#include "motor.h"

/* motor velocity targets */
float left_vel_target_meter_per_sec  = 0.0;
float right_vel_target_meter_per_sec = 0.0;

void set_wheel_target_velocity(float left, float right) {
  left_vel_target_meter_per_sec = left;
  right_vel_target_meter_per_sec = right;
}

float get_left_wheel_target_velocity() {
  return left_vel_target_meter_per_sec;
}

float get_right_wheel_target_velocity() {
  return right_vel_target_meter_per_sec;
}

void set_twist_target(float linear_m_s, float angle_rad_s) {
  // convert twist input from pi to motor velocities.
  float right_vel = (angle_rad_s * WHEEL_SEPERATION_DIST_M) / 2.0 + linear_m_s;
  float left_vel = (linear_m_s * 2.0) - right_vel;

  set_wheel_target_velocity( left_vel, right_vel );
}
