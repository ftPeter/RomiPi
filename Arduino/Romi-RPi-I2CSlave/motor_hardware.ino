/* motor_hardware
 *  
 * hardware interface methods
 * 
 * NOTE: the robot velocity is controlled per-wheel
 *       velocity instead of for the twist. twist
 *       would be preferable and may be changed in
 *       future revisions.
 */

#include "motor.h"

/* interface to allow swapping motor directions
    and positions.

   note: this might be doable directly with the pololu motors object
*/
int16_t debug_left_motor_power, debug_right_motor_power;

int16_t debug_get_left_motor_power() { 
  return debug_left_motor_power;
}
int16_t debug_get_right_motor_power() {
  return debug_right_motor_power;
}

void hw_motors_setspeeds(int16_t left, int16_t right) {
  int16_t left_motor = left * FLIP_MOTOR_DIR;
  int16_t right_motor = right * FLIP_MOTOR_DIR;
  if ( SWAP_MOTORS ) {
    debug_left_motor_power = left_motor;
    debug_right_motor_power = right_motor;
    motors.setSpeeds(left_motor, right_motor);
  } else {
    debug_left_motor_power = right_motor;
    debug_right_motor_power = left_motor;
    motors.setSpeeds(right_motor, left_motor);
  }
}

/* interface to allow swapping motor directions
    and positions.
*/

int16_t hw_getencoder_left() {
  if ( SWAP_ENCODERS ) {
    return encoders.getCountsRight() * FLIP_ENCODER_DIR;
  } else {
    return encoders.getCountsLeft() * FLIP_ENCODER_DIR;
  }
}

int16_t hw_getencoder_right() {
  if ( SWAP_ENCODERS ) {
    return encoders.getCountsLeft() * FLIP_ENCODER_DIR;
  } else {
    return encoders.getCountsRight() * FLIP_ENCODER_DIR;
  }
}
