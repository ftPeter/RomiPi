#include <Servo.h>
#include <Romi32U4.h>
#include <PololuRPiSlave.h>

#define ROMI_FIRMWARE_VERSION (11)

/* This example program shows how to make the Romi 32U4 Control Board
   into a Raspberry Pi I2C slave.  The RPi and Romi 32U4 Control Board can
   exchange data bidirectionally, allowing each device to do what it
   does best: high-level programming can be handled in a language such
   as Python on the RPi, while the Romi 32U4 Control Board takes charge
   of motor control, analog inputs, and other low-level I/O.

   Make sure to set the Sketchbook location to
   ../RomiPi/sketch_arduino

   The example and libraries are available for download at:

   https://github.com/pololu/pololu-rpi-slave-arduino-library

   You will need the corresponding Raspberry Pi code, which is
   available in that repository under the pi/ subfolder.  The Pi code
   sets up a simple Python-based web application as a control panel
   for your Raspberry Pi robot.
*/

// Custom data structure that we will use for interpreting the buffer.
// We recommend keeping this under 64 bytes total.  If you change the
// data format, make sure to update the corresponding code in
// a_star.py on the Raspberry Pi.

struct Data
{
  uint8_t romi_fw_version = ROMI_FIRMWARE_VERSION; // 0
  bool green, red; // 1,2
  bool buttonA, buttonB, buttonC; // 3,4,5

  float left_vel_target_meter_per_sec, right_vel_target_meter_per_sec; // 6,7,8,9, 10,11,12,13
  uint16_t analog[4]; // 14,15, 16,17, 18,19, 20,21

  bool resetEncoders; // 24
  bool fillSpace; // 25
  int16_t leftEncoder, rightEncoder; // 26,27, 28,29

  float pose_x, pose_y;     // 30,31,32,33, 34,35,36,37,
  float pose_quat_z, pose_quat_w;   // 38,39,40,41, 42,43,44,45,
  float pose_twist_linear_x, pose_twist_angle_z; //  46,47,48,49, 50,51,52,53,
  uint16_t batteryMillivolts;  // 54,55
  bool yellow; // 56

};

PololuRPiSlave<struct Data, 20> slave;
PololuBuzzer buzzer;
Romi32U4Motors motors;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4Encoders encoders;

/* PREVIOUS TIME AND ENCODER VALUES */
unsigned long last_time_ms = 0;
int prev_left_count_ticks, prev_right_count_ticks;
float left_vel_meter_per_sec, right_vel_meter_per_sec;

/* CURRENT POSE */
float pose_x_m = 0;
float pose_y_m = 0;
float pose_th_rad = 0;
float pose_quat_z_unitless = 0;
float pose_quat_w_unitless = 0;
float pose_twist_linear_x_m_per_s = 0;
float pose_twist_angle_z_rad_per_s  = 0;

/* MOTOR CONTROL */
float left_vel_target_meter_per_sec  = 0.0;
float right_vel_target_meter_per_sec = 0.0;
int16_t left_motor, right_motor;

void setup()
{
  // Set up the slave at I2C address 20.
  slave.init(20);

  // Play startup sound.
  buzzer.play("v10>>g16>>>c16");
}

void loop()
{
  // dump this after debugging
  /*
    char buf[64];
    Serial.print(" slave.buffer 0x");
    sprintf(buf, "%p", &(slave.buffer));
    Serial.println(buf);
    for (int i = 0; i < 54; i++) {
    sprintf(buf, "%x ", ((uint8_t*)(&slave.buffer))[i]);
    Serial.print(buf);
    }
    Serial.println("");
  
  Serial.print("l: ");
  Serial.print(slave.buffer.leftEncoder);
  Serial.print(", r: ");
  Serial.print(slave.buffer.rightEncoder);
  Serial.print(" ");
  */


  // Call updateBuffer() before using the buffer, to get the latest
  // data including recent master writes.
  slave.updateBuffer();

  // Write various values into the data structure.
  slave.buffer.buttonA = buttonA.isPressed();
  slave.buffer.buttonB = buttonB.isPressed();
  slave.buffer.buttonC = buttonC.isPressed();

  // Change this to readBatteryMillivoltsLV() for the LV model.
  // slave.buffer.batteryMillivolts = readBatteryMillivolts();

  for (uint8_t i = 0; i < 4; i++)
  {
    slave.buffer.analog[i] = analogRead(i);
  }

  // READING the buffer is allowed before or after finalizeWrites().
  ledYellow(slave.buffer.yellow);
  ledGreen(slave.buffer.green);
  ledRed(slave.buffer.red);

  slave.buffer.leftEncoder = encoders.getCountsLeft();
  slave.buffer.rightEncoder = encoders.getCountsRight();

  if (slave.buffer.resetEncoders)
  {
    slave.buffer.resetEncoders = 0;
    slave.buffer.leftEncoder   = encoders.getCountsAndResetLeft();
    slave.buffer.rightEncoder  = encoders.getCountsAndResetRight();
  }

  if ( everyNmillisec(10) ) {
    // ODOMETRY
    calculateOdom();
    slave.buffer.pose_x              = pose_x_m;
    slave.buffer.pose_y              = pose_y_m;
    slave.buffer.pose_quat_z         = pose_quat_z_unitless;
    slave.buffer.pose_quat_w         = pose_quat_w_unitless;
    slave.buffer.pose_twist_linear_x = pose_twist_linear_x_m_per_s;
    slave.buffer.pose_twist_angle_z  = pose_twist_angle_z_rad_per_s;

    left_vel_target_meter_per_sec  = slave.buffer.left_vel_target_meter_per_sec;
    right_vel_target_meter_per_sec = slave.buffer.right_vel_target_meter_per_sec;
    doPID();
  }

  if ( every100millisec() ) {
    debug_motors();
  }

  // When you are done WRITING, call finalizeWrites() to make modified
  // data available to I2C master.
  slave.finalizeWrites();

  /*
    Serial.print("x: ");
    Serial.print(slave.buffer.pose_x);
    Serial.print(", y: ");
    Serial.print(slave.buffer.pose_y);
    Serial.print(", z: ");
    Serial.print(slave.buffer.pose_quat_z);
    Serial.print(", w: ");
    Serial.print(slave.buffer.pose_quat_w);
    Serial.print(", vel x_m: ");
    Serial.print(slave.buffer.pose_twist_linear_x);
    Serial.print(", vel z_ang: ");
    Serial.print(slave.buffer.pose_twist_angle_z);
    Serial.println(" ");
  */
}
