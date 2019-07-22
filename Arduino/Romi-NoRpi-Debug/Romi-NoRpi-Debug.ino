#include <Romi32U4.h>
#include <PololuRPiSlave.h>

Romi32U4Motors motors;
Romi32U4Encoders encoders;
Romi32U4ButtonA buttonA;

void setup() {
  Serial.begin(57600);

  // put your setup code here, to run once:
  ledYellow(false);
  ledGreen(true);
  ledRed(false);
}

float _debug_linear_ms = 0.25;
float _debug_angle_rs = 0.0;
void _DEBUG_PID_CONTROL() {
  static float _linear_ms_change = 0.1;

  
  set_twist_target(_debug_linear_ms, _debug_angle_rs);
}

void loop() {


  // put your main code here, to run repeatedly:
  if (everyNmillisec(10)) {
    // ODOMETRY
    calculateOdom();
    doPID();
  }

}
