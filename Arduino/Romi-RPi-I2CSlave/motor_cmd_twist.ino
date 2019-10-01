
/* motor_cmd_twist

   methods for handling twist commands and converting them
    to target motor velocities

   the deadman's switch/handle is also written here.
   the deadman's handle stops the robot if a command
   has not been received from the raspberry pi
   every DEADMAN_TIMEOUT_MS milliseconds.

   November 24, 2018
*/

#include "motor.h"

/* motor velocity targets */
float left_vel_target_meter_per_sec  = 0.0;
float right_vel_target_meter_per_sec = 0.0;


/* timeout */
unsigned long _deadman_timeout_timer = 0;

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

void _set_twist_target(float linear_m_s, float angle_rad_s) {
  // convert twist input from pi to motor velocities.
  float right_vel = (angle_rad_s * WHEEL_SEPERATION_DIST_M) / 2.0 + linear_m_s;
  float left_vel = (linear_m_s * 2.0) - right_vel;

  set_wheel_target_velocity( left_vel, right_vel );
}

// _deadman_timeout_reset
//    resets the timeout for the motor control
void _deadman_timeout_reset() {
  _deadman_timeout_timer = millis();
}

// _deadman_timeout_isTimedOut
// returns true if the elapsed time since the last
// new robot control update was more than DEADMAN_TIMEOUT_MS
// and returns false if a new control signal is returned.
bool _deadman_timeout_isTimedOut() {
  bool ret_error = ((millis() - _deadman_timeout_timer) > DEADMAN_TIMEOUT_MS);
#ifdef DBG_SRL
  if ( ret_error ) {
    Serial.print("control_timeout_isTimedOut: ");
    Serial.print(millis());
    Serial.print(", ");
    Serial.print(_deadman_timeout_timer);
    Serial.println("");
  }
#endif
  return ret_error;
}

void set_twist_target( float *linear, float *angular, bool *new_twist) {
  // set twist sent from Raspberry Pi
  // note: we stop the robot if we haven't
  //       heard from the Raspberry Pi in 100 ms
  if (*new_twist == true) {
    // reset control timeout
    _deadman_timeout_reset();
    *new_twist = false;
    _set_twist_target(*linear, *angular);
  } else if (_deadman_timeout_isTimedOut()) {
    _set_twist_target(0.0, 0.0);
  }
}
