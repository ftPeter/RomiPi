/*  PID controller inspired by:
     https://github.com/jfstepha/differential-drive/blob/master/scripts/pid_velocity.py
    Planned work:
 ** Add reverse motor capability (also to buffer)
 ** Add left_right swap capability (also to buffer)
 ** change to moving window average velocity
    instead of instantaneous
 ** add encoder wrap capability to velocity calculation
 ** unify odometry and odometry calculation
*/

const int window_size = 10;
float left_vel_window[window_size];
float right_vel_window[window_size];

float left_wheel_velocity() {
  // average left wheel velocity
  // in meters per second
  float average = 0;
  static int i;
  left_vel_window[i++] = left_vel_meter_per_sec;
  if( i >= window_size ) { i = 0; } 
  for(int j = 0; j < window_size; j++) {
    average += left_vel_window[i];
  }
  average /= window_size;
  
  return average;
}

float right_wheel_velocity() {
  // average right wheel velocity
  // in meters per second
  float average = 0;
  static int i;
  right_vel_window[i++] = right_vel_meter_per_sec;
  if( i >= window_size ) { i = 0; }
  for(int j = 0; j < window_size; j++) {
    average += right_vel_window[i];
  }
  average /= window_size;
  return average;
}

float get_left_wheel_velocity_target() {
  // target left wheel velocity
  // in meters per second
  return left_vel_target_meter_per_sec;
}

float get_right_wheel_velocity_target() {
  // target left wheel velocity
  // in meters per second
  return right_vel_target_meter_per_sec;
}

void set_twist_target(float linear_m_s, float angle_rad_s) {
  // convert twist input from pi to motor velocities.
  // TODO unsure if this is correct, double check unit analysis
  float wheel_dist_m = 0.14;
  
  right_vel_target_meter_per_sec = (angle_rad_s * wheel_dist_m * 100.0) / 2.0 + linear_m_s;
  left_vel_target_meter_per_sec = (linear_m_s * 2.0) - right_vel_target_meter_per_sec;
}

void doPID() {
  /* PID AND MOTOR CONSTANTS */
  const int MOTOR_MAX = 300, MOTOR_MIN = -300;
  const float Kp = 1000;
  const float Ki = 1000;
  const float Kd = 0.01;

  /* STATIC VARIABLES */
  static unsigned long prev_time_ms;
  static float left_prev_error, right_prev_error;
  static float left_integral, right_integral;

  /* TIME DURATION SINCE LAST UPDATE */
  unsigned long current_time_ms = millis();
  long duration_ms = current_time_ms - prev_time_ms;
  float duration_s = 1000.0 * float(duration_ms);

  /* CALCULATE ERROR */
  float left_error  = get_left_wheel_velocity_target() - left_wheel_velocity();
  float right_error = get_right_wheel_velocity_target() - right_wheel_velocity();

  /* CALCULATE PID */
  left_integral  = left_integral  + (left_error * duration_s);
  right_integral = right_integral + (right_error * duration_s);
  float left_derivative  = (left_error - left_prev_error) / duration_s;
  float right_derivative = (right_error  - right_prev_error) / duration_s;

  left_motor  = int16_t(Kp * left_error  + Ki * left_integral  + Kd * left_derivative);
  right_motor = int16_t(Kp * right_error + Ki * right_integral + Kd * right_derivative);

  /* CONSTRAIN MOTOR SETTING RANGE */
  if ( left_motor > MOTOR_MAX ) {
    left_motor = MOTOR_MAX;
    left_integral  = left_integral - (left_error * duration_s);
  } else if ( left_motor < MOTOR_MIN ) {
    left_motor = MOTOR_MIN;
    left_integral  = left_integral - (left_error * duration_s);
  }
  
  if ( right_motor > MOTOR_MAX ) {
    right_motor = MOTOR_MAX;
    right_integral  = right_integral - (right_error * duration_s);
  } else if ( right_motor < MOTOR_MIN ) {
    right_motor = MOTOR_MIN;
    right_integral  = right_integral - (right_error * duration_s);
  }

  if ( get_left_wheel_velocity_target() == 0.0 ) {
    left_motor = 0;
  }
  if ( get_right_wheel_velocity_target() == 0.0 ) {
    right_motor = 0;
  }

  /* SET THE MOTORS */
  //motors.setSpeeds(left_motor, right_motor);
  
  // update previous values
  prev_time_ms = current_time_ms;
  left_prev_error  = left_error;
  right_prev_error = right_error;
}


void debug_motors() {
  Serial.print("left");
  Serial.print(" pow "); Serial.print(left_motor);
  Serial.print(", inst_v "); Serial.print(left_wheel_velocity());
  Serial.print(", target_v "); Serial.print(get_left_wheel_velocity_target());
  Serial.print(" right");
  Serial.print(" pow "); Serial.print(right_motor);
  Serial.print(", inst_v "); Serial.print(right_wheel_velocity());
  Serial.print(", target_v "); Serial.print(get_right_wheel_velocity_target());
  Serial.println();
}
