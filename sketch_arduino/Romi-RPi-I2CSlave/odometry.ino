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

float calculate_velocity(current_ticks, prev_ticks, dt_s) {
  float delta_ticks = current_ticks - prev_ticks;
  return delta_ticks * METERS_PER_TICK / dt_s;
}

/* CALCULATE THE ODOMETRY */
void calculateOdom() {
  /* READ THE ENCODERS AND TIME */
  unsigned long cur_time_ms = millis();
  int left_count_ticks  = hw_getencoder_left();
  int right_count_ticks = hw_getencoder_right();

  //extract the wheel velocities from the tick signals count
  float delta_left_ticks  = float(left_count_ticks - prev_left_count_ticks);
  float delta_right_ticks = float(right_count_ticks - prev_right_count_ticks);

  float dt_s = elapsed_seconds(cur_time_ms, last_time_ms);

  /* CALCULATE THE CURRENT VELOCITY */
  pose_left_vel_meter_per_sec  = (delta_left_ticks  * METERS_PER_TICK) / dt_s;
  pose_right_vel_meter_per_sec = (delta_right_ticks * METERS_PER_TICK) / dt_s;

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
  pose_th_rad += delta_th_rad;

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
