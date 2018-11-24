/*  PID controller inspired by:
     https://github.com/jfstepha/differential-drive/blob/master/scripts/pid_velocity.py
*/


/* methods for handling twist commands and converting them 
 *  to target motor velocities
 */

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

/* methods smoothing instantaneous wheel velocities reported by
 * the odometry methods get_instant_left_wheel_vel(), get_instant_right_wheel_vel()
 */

/* arrays for calculating moving window average wheel velocity */
const int window_size = 10;
float left_vel_window[window_size];
float right_vel_window[window_size];

float get_left_average_wheel_velocity() {
  // average instantaneous left wheel velocity
  // in meters per second
  float average = 0;
  static int i;
  left_vel_window[i++] = get_instant_left_wheel_vel();
  if( i >= window_size ) { i = 0; }
  for(int j = 0; j < window_size; j++) {
    average += left_vel_window[i];
  }
  average /= window_size;
  return average;
}

float get_right_average_wheel_velocity() {
  // average instantaneous right wheel velocity
  // in meters per second
  float average = 0;
  static int i;
  right_vel_window[i++] = get_instant_right_wheel_vel();
  if ( i >= window_size ) { i = 0;}
  for (int j = 0; j < window_size; j++) {
    average += right_vel_window[i];
  }
  average /= window_size;
  return average;
}

/*  setMotorSpeeds
 *
 *  constraint the motor speed within the motor power limit
 *  then set the speeds
 */
void setMotorSpeeds(int16_t left_motor, int16_t right_motor,
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
  /* PID AND MOTOR CONSTANTS */
  const float Kp = 300;
  const float Ki = 0.0; // 1000;
  const float Kd = 0.0; // 0.01;

  /* STATIC VARIABLES */
  static unsigned long prev_time_ms;
  static float left_prev_error, right_prev_error;
  static float left_integral, right_integral;

  /* TIME DURATION SINCE LAST UPDATE */
  unsigned long current_time_ms = millis();
  long duration_ms = current_time_ms - prev_time_ms;
  float duration_s = 1000.0 * float(duration_ms);

  /* CALCULATE ERROR : error = target - measured */
  float left_error  = get_left_wheel_target_velocity() - get_left_average_wheel_velocity();
  float right_error = get_right_wheel_target_velocity() - get_right_average_wheel_velocity();

  /* PID CALCULATIONS*/
  left_integral  = left_integral  + (left_error * duration_s);
  right_integral = right_integral + (right_error * duration_s);
  float left_derivative  = (left_error - left_prev_error) / duration_s;
  float right_derivative = (right_error - right_prev_error) / duration_s;

  int16_t left_motor  = int16_t(Kp * left_error  + Ki * left_integral  + Kd * left_derivative);
  int16_t right_motor = int16_t(Kp * right_error + Ki * right_integral + Kd * right_derivative);

  setMotorSpeeds(left_motor, right_motor,
                 left_error, right_error,
                 &left_integral, &right_integral,
                 duration_s);

  // update previous values
  prev_time_ms = current_time_ms;
  left_prev_error  = left_error;
  right_prev_error = right_error;
}
