/* CALCULATE THE ODOMETRY */
void calculateOdom() {
  /* ODOMETRY CONSTANTS */
  // wheel diameter in meters (70 mm)
  float wheel_diam_m = 0.07;
  //the distance between two wheels in meters (14 cm)
  float wheel_seperation_dist_m = 0.14;
  // calculate the distance per tick
  float dist_per_count_m_per_tick = (3.14159 * wheel_diam_m) / 1440.0;

  /* READ THE ENCODERS AND TIME */
  unsigned long cur_time_ms = millis();
  int left_count_ticks  = encoders.getCountsLeft();
  int right_count_ticks = encoders.getCountsRight();
  
  //extract the wheel velocities from the tick signals count
  float delta_left_ticks  = float(left_count_ticks - prev_left_count_ticks);
  float delta_right_ticks = float(right_count_ticks - prev_right_count_ticks);

  float dt_s = float(cur_time_ms - last_time_ms) / 1000.0;

  /* CALCULATE THE CURRENT VELOCITY */
  left_vel_meter_per_sec  = (delta_left_ticks  * dist_per_count_m_per_tick) / dt_s;
  right_vel_meter_per_sec = (delta_right_ticks * dist_per_count_m_per_tick) / dt_s;

  float vx_m_per_s = ((left_vel_meter_per_sec + right_vel_meter_per_sec) / 2);
  float vth_rad_per_s = ((left_vel_meter_per_sec - right_vel_meter_per_sec) / wheel_seperation_dist_m);

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
  pose_twist_linear_x_m_per_s   = vx_m_per_s;
  pose_twist_angle_z_rad_per_s  = vth_rad_per_s;

  //update previous values
  prev_left_count_ticks  = left_count_ticks;
  prev_right_count_ticks = right_count_ticks;
  last_time_ms = cur_time_ms;
}
