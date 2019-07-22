/* timing
 * 
 * functions related to time.
 * 
 * November 24, 2018
 */

float elapsed_seconds(unsigned long current_ms, unsigned long previous_ms) {
  // TODO update this to account for roll-over (after 50 days)
  return ms_to_seconds(float(current_ms - previous_ms));
}

float ms_to_seconds(float ms) {
  return ms / 1000.0;
}

bool everyNmillisec(unsigned int N) {
  static unsigned int next_time_ms;
  unsigned int current_time_ms = millis();
  if(current_time_ms < next_time_ms) {
    return false;
  }
  next_time_ms = current_time_ms + N;
  return true;
}

bool every100millisec() {
  static unsigned int next_time_ms;
  unsigned int current_time_ms = millis();
  if(current_time_ms < next_time_ms) {
    return false;
  }
  next_time_ms = current_time_ms + 100;
  return true;
}
