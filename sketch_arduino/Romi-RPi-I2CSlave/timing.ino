
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
