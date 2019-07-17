/* odometry

   functions related to measuring pose.

   November 24, 2018
*/

/* CURRENT POSE */
float pose_x_m = 0;
float pose_y_m = 0;
float pose_th_rad = 0;
float pose_quat_z_unitless = 0;
float pose_quat_w_unitless = 0;
float pose_twist_linear_x_m_per_s = 0.0;
float pose_twist_angle_z_rad_per_s = 0.0;
float pose_left_vel_meter_per_sec = 0.0;
float pose_right_vel_meter_per_sec = 0.0;

float get_pose_x() {
  return pose_x_m;
}

float get_pose_y() {
  return pose_y_m;
}

float get_pose_th_rad() {
  return pose_th_rad;
}

float get_pose_quat_z() {
  return pose_quat_z_unitless;
}

float get_pose_quat_w() {
  return pose_quat_w_unitless;
}

float get_pose_twist_linear() {
  return pose_twist_linear_x_m_per_s;
}

float get_pose_twist_angle() {
  return pose_twist_angle_z_rad_per_s;
}

float get_instant_left_wheel_vel() {
  return pose_left_vel_meter_per_sec;
}

float get_instant_right_wheel_vel() {
  return pose_right_vel_meter_per_sec;
}

float calculate_velocity(int16_t current_ticks, int16_t prev_ticks, float dt_s) {
  if ( dt_s == 0.0 ) {
    return -1.0;
  }

  float delta_ticks;
  // Worry about three cases for tick difference:
  if ( prev_ticks < ENCODER_WIN_LOW and current_ticks > ENCODER_WIN_HIGH) {
    // 1. roll-over from previous near max and current near min
    delta_ticks = MIN_INT16 - prev_ticks;
    delta_ticks += current_ticks - MAX_INT16;
  } else if (prev_ticks > ENCODER_WIN_HIGH and current_ticks < ENCODER_WIN_LOW) {
    // 2. roll-over from previous near max and current near min
    delta_ticks = MAX_INT16 - prev_ticks;
    delta_ticks += current_ticks - MIN_INT16;
  } else {
    // 3. no roll-over
    delta_ticks = float(current_ticks - prev_ticks);
  }

  return (delta_ticks * METERS_PER_TICK / dt_s);
}

/* CALCULATE THE ODOMETRY */
void calculateOdom() {
  /* PREVIOUS TIME AND ENCODER VALUES */
  static unsigned long last_time_ms;
  static int16_t prev_left_count_ticks, prev_right_count_ticks;

  /* READ THE ENCODERS AND TIME */
  unsigned long cur_time_ms = millis();
  float dt_s = elapsed_seconds(cur_time_ms, last_time_ms);
  int16_t left_count_ticks  = hw_getencoder_left();
  int16_t right_count_ticks = hw_getencoder_right();

  /* CALCULATE THE CURRENT VELOCITY */
  pose_left_vel_meter_per_sec  = calculate_velocity(left_count_ticks, prev_left_count_ticks, dt_s);
  pose_right_vel_meter_per_sec = calculate_velocity(right_count_ticks, prev_right_count_ticks, dt_s);

  float vx_m_per_s = ((pose_left_vel_meter_per_sec + pose_right_vel_meter_per_sec) / 2);
  float vth_rad_per_s = ((pose_right_vel_meter_per_sec - pose_left_vel_meter_per_sec) / WHEEL_SEPERATION_DIST_M);

  /* CALCULATE THE CHANGE SINCE PREVIOUS CALL */
  float delta_x_m  = (vx_m_per_s * cos(pose_th_rad)) * dt_s;
  float delta_y_m  = (vx_m_per_s * sin(pose_th_rad)) * dt_s;
  float delta_th_rad = vth_rad_per_s * dt_s;

  /* UPDATE THE CURRENT POSITION */
  // update the current position
  pose_x_m  += delta_x_m;
  pose_y_m  += delta_y_m;
  pose_th_rad = normalize_angle(pose_th_rad + delta_th_rad);

  pose_quat_z_unitless = sin(pose_th_rad * 0.5);
  pose_quat_w_unitless = cos(pose_th_rad * 0.5);

  //set the current velocity
  pose_twist_linear_x_m_per_s  = vx_m_per_s;
  pose_twist_angle_z_rad_per_s = vth_rad_per_s;

  //update previous values
  prev_left_count_ticks  = left_count_ticks;
  prev_right_count_ticks = right_count_ticks;
  last_time_ms = cur_time_ms;
}
