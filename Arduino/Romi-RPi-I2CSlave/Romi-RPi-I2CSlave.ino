//#include <Servo.h>
#include <Romi32U4.h>
#include <PololuRPiSlave.h>

#define ROMI_FIRMWARE_VERSION (14)

/* Pololu Romi Example Code included in here:

   This example program shows how to make the Romi 32U4 Control Board
   into a Raspberry Pi I2C slave.  The RPi and Romi 32U4 Control Board can
   exchange data bidirectionally, allowing each device to do what it
   does best: high-level programming can be handled in a language such
   as Python on the RPi, while the Romi 32U4 Control Board takes charge
   of motor control, analog inputs, and other low-level I/O.

   Make sure to set the Sketchbook location to
   ../RomiPi/Arduino

*/

/* Custom data structure that we will use for interpreting the buffer.
   We recommend keeping this under 64 bytes total.  If you change the
   data format, make sure to update the corresponding code in
   a_star.py on the Raspberry Pi.

   If you change the struct, make sure to update the FIRMWARE VERSION

   The number written after each variable declaration is the byte offset
   from the start of the buffer. Refer to these when writing the
   Raspberry Pi-side driver.
*/
struct Data
{
  uint8_t romi_fw_version = ROMI_FIRMWARE_VERSION; // 0
  bool led_green, led_red, led_yellow; // 1,2,3
  uint8_t pixel_red, pixel_green, pixel_blue; // 4,5,6
  bool buttonA, buttonB, buttonC; // 7,8,9
  uint16_t batteryMillivolts;  // 10,11

  //uint16_t analog[4];

  // encoders sent to PI
  bool resetEncoders; // 12
  int16_t leftEncoder, rightEncoder; // 13,14,  15,16

  // pose state sent to PI
  float pose_x, pose_y, pose_th_rad;     // 17,18,19,20,   21,22,23,24,   25,26,27,28,
  float pose_quat_z, pose_quat_w;   //  29,30,31,32,   33,34,35,36,
  float pose_twist_linear_x, pose_twist_angle_z; // 37,38,39,40,   41,42,43,44,
  float pose_left_vel_target_meter_per_sec, pose_right_vel_target_meter_per_sec; //   45,46,47,48, 49,50,51,52,

  // twist setting from PI
  float twist_linear_x, twist_angle_z; // 53,54,55,56,   57, 58, 59, 60
};

// PololuRPiSlave<BufferType, pi_delay_us>
// NOTE: the 20 us delay here and the address of 20 below
//       are just a coincidence, not a typo.
PololuRPiSlave<struct Data, 20> slave;
PololuBuzzer buzzer;
Romi32U4Motors motors;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4Encoders encoders;



void setup()
{
  Serial.begin(57600);

  // Set up the slave at I2C address 20.
  // NOTE: the address of 20 below and 20 us delay above 
  //       are just a coincidence, not a typo.
  slave.init(20);

  // Play startup sound.
  buzzer.play("v10>>e16>>>g16");

  lights_init();

  ledYellow(false);
  ledGreen(true);
  ledRed(false);
}

void loop()
{
  // Call updateBuffer() before using the buffer, to get the latest
  // data including recent master writes.
  slave.updateBuffer();

  // set twist sent from Raspberry Pi
  set_twist_target(slave.buffer.twist_linear_x,
                   slave.buffer.twist_angle_z);

  // Write various values into the data structure.
  slave.buffer.buttonA = buttonA.isPressed();
  slave.buffer.buttonB = buttonB.isPressed();
  slave.buffer.buttonC = buttonC.isPressed();

  // Change this to readBatteryMillivoltsLV() for the LV model.
  slave.buffer.batteryMillivolts = readBatteryMillivolts();

  // update LED signals
  ledYellow(slave.buffer.led_yellow);
  ledGreen(slave.buffer.led_green);
  ledRed(slave.buffer.led_red);

  lights_update(slave.buffer.pixel_red,
                slave.buffer.pixel_green,
                slave.buffer.pixel_blue);

  // update encoders
  if (slave.buffer.resetEncoders)
  { // reset and update encoder buffer
    slave.buffer.resetEncoders = 0;
    slave.buffer.leftEncoder   = encoders.getCountsAndResetLeft();
    slave.buffer.rightEncoder  = encoders.getCountsAndResetRight();
  } else {
    // update encoder buffer without reset
    slave.buffer.leftEncoder = hw_getencoder_left();
    slave.buffer.rightEncoder = hw_getencoder_right();
  }

  if (everyNmillisec(10)) {
    // ODOMETRY
    calculateOdom();
    doPID();

    // measured orientation of robot on X,Y plane
    slave.buffer.pose_x              = get_pose_x();
    slave.buffer.pose_y              = get_pose_y();
    slave.buffer.pose_th_rad         = get_pose_th_rad();
    slave.buffer.pose_quat_z         = get_left_wheel_target_velocity(); //= get_pose_quat_z(); TODO DEBUG HACK
    slave.buffer.pose_quat_w         = get_right_wheel_target_velocity();//= get_pose_quat_w(); TODO DEBUG HACK
    // measured twist
    slave.buffer.pose_twist_linear_x = get_pose_twist_linear();
    slave.buffer.pose_twist_angle_z  = get_pose_twist_angle();
    // measured wheel velocities
    slave.buffer.pose_left_vel_target_meter_per_sec  = get_instant_left_wheel_vel();
    slave.buffer.pose_right_vel_target_meter_per_sec = get_instant_right_wheel_vel();

    
  }
      // READING the buffer is allowed before or after finalizeWrites().
  // When you are done WRITING, call finalizeWrites() to make modified
  // data available to I2C master.
  slave.finalizeWrites();

  // COMMENT THIS OUT AFTER DEBUG
  if (every100millisec()) {
    Serial.print("ODOMETRY DEBUG...");

    Serial.print("L: ");
    Serial.print(get_instant_left_wheel_vel());
    Serial.print(" m/s, R: ");
    Serial.print(get_instant_right_wheel_vel());
    Serial.print(" m/s. ");

    Serial.print("(x: "); Serial.print(get_pose_x());
    Serial.print("m, y: "); Serial.print(get_pose_y());
    Serial.print("m, t: "); Serial.print(get_pose_th_rad());
    Serial.print(" rad). ");

    Serial.print("("); Serial.print(get_pose_twist_linear());
    Serial.print(" m/s, "); Serial.print(get_pose_twist_angle());
    Serial.print("rad/s ).");

    Serial.print("PID ");
    Serial.print(get_left_wheel_target_velocity());
    Serial.print(", ");
    Serial.print(get_right_wheel_target_velocity());
    Serial.print("");

    Serial.println("");
  }

}
