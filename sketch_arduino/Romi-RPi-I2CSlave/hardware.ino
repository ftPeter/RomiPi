/* ROMI SPECIFIC CONSTANTS */
//the distance between two wheels in meters (14 cm)
const float WHEEL_SEPERATION_DIST_M = 0.14;
// wheel diameter in meters (7 cm)
const float WHEEL_DIAM_M = 0.07;
// calculate the distance per tick
const float TICKS_PER_ROTATION = 1440.0;
float METERS_PER_TICK = (PI * WHEEL_DIAM_M) / TICKS_PER_ROTATION;

/* MOTOR RELATED CONSTANTS */
const int MOTOR_MAX = 300, MOTOR_MIN = -300;

/* interface to allow swapping motor directions
    and positions.

   note: this might be doable directly with the pololu motors object
*/
const int flip_motor_dir = -1;
const bool swap_motors = false;

int debug_left_motor_power, debug_right_motor_power;

int debug_get_left_motor_power() { 
  return debug_left_motor_power;
}
int debug_get_right_motor_power() {
  return debug_right_motor_power;
}

void hw_motors_setspeeds(int left, int right) {
  int left_motor = left * flip_motor_dir;
  int right_motor = right * flip_motor_dir;
  if ( swap_motors ) {
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
const int16_t MAX_INT16 = 32767;
const int16_t MIN_INT16 = -32768;
const int16_t ENCODER_WIN_HIGH = MAX_INT16 * 0.7;
const int16_t ENCODER_WIN_LOW  = MIN_INT16 * 0.7;

int flip_encoder_dir = -1;
bool swap_encoders = false;

int hw_getencoder_left() {
  if ( swap_encoders ) {
    return encoders.getCountsRight() * flip_encoder_dir;
  } else {
    return encoders.getCountsLeft() * flip_encoder_dir;
  }
}

int hw_getencoder_right() {
  if ( swap_encoders ) {
    return encoders.getCountsLeft() * flip_encoder_dir;
  } else {
    return encoders.getCountsRight() * flip_encoder_dir;
  }
}

