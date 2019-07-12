/* motor_hardware
 *  
 * hardware interface methods
 * 
 */

#include "motor.h"

/* interface to allow swapping motor directions
    and positions.

   note: this might be doable directly with the pololu motors object
*/
int debug_left_motor_power, debug_right_motor_power;

int debug_get_left_motor_power() { 
  return debug_left_motor_power;
}
int debug_get_right_motor_power() {
  return debug_right_motor_power;
}

void hw_motors_setspeeds(int left, int right) {
  int left_motor = left * FLIP_MOTOR_DIR;
  int right_motor = right * FLIP_MOTOR_DIR;
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
