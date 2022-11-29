#include <Alfredo_NoU2.h>
#include <AlfredoConnect.h>
#include <BluetoothSerial.h>
#include <Adafruit_BNO08x.h>
#include <PID_v1.h>  // https://github.com/br3ttb/Arduino-PID-Library/blob/master/examples/PID_Basic/PID_Basic.ino

//Gyro Setup
//
Adafruit_BNO08x gyro(-1);  // Might need the reset pin to be 5
sh2_SensorValue_t quat;    //https://github.com/adafruit/Adafruit_BNO08x/blob/67b91b809da04a08fccb8793770343f872daaf43/src/sh2_SensorValue.h#L186
struct euler_t {
  double yaw;
  double pitch;
  double roll;
} rot;

// NoU Setup
//
NoU_Motor right_drive(1);
NoU_Motor left_drive(2);
NoU_Drivetrain drivetrain(&left_drive, &right_drive);

NoU_Motor right_shoot(3);
NoU_Motor left_shoot(4);

NoU_Motor intake(5);

NoU_Servo arm_servo(1);
NoU_Servo defense_servo(2);


//Servo positions
//
#define arm_intake 155
#define arm_closeshot 20
#define arm_farshot 120

#define defense_up 0
#define defense_chival 90
#define defense_down 180


//Controls Setup
//
float joy_throttle = 0.0;     // raw joystick throttle input
float joy_steer = 0.0;        // raw joystick steering input
int joy_arm = 0;              // button state of joystick arm control
int joy_defense = 0;          // button state of joystick defense arm control
bool joy_reverse = false;     // button state of joystick reverse-drive control

bool PID_steering = false;    // determins if the joystick is steering or if the PID loop is steering
bool reverse_drive = true;  // true = shooter forward
int arm_position = 1;         // 0, 1, 2 == intake, far shot, close shot
int defense_position = 1;     // 0, 1, 2 == down, chival, up
long shooter_time = 0;        // Timer for shooter flywheel spoolup

//Starting yaw/pitch is always 0/0
#define drive_degrees_per_second 60.0
double last_yaw = 0.0;     // last yaw reading, used to keep track of cumulative yaw
int rotations = 0;         // keeps track of # of rotations the bot has made in either direction
double PID_input = 0.0;    // cumulative yaw. wraps around when passing 180° so that the PID loop doesn't get confused
double PID_setpoint;       // desired_yaw + cum_yaw
double PID_output = 0.0;   // Result of the PID computation, used to steer

double desired_yaw = 0.0;  // Desired facing direction, as modified by the joystick, global angles



PID steer_loop(&PID_input, &PID_output, &PID_setpoint, 0.2, 0.0, 0.0, DIRECT); //P 1.0, I 0.1, D 0.5

//Code State
//
long last_time = millis();
long debug_timer = 0;


BluetoothSerial bluetooth;

void setup() {
  //Misc setup
  steer_loop.SetOutputLimits(-1.0, 1.0);
  intake.setInverted(true);
  
  //LED
  RSL::initialize();
  RSL::setState(RSL_DISABLED);

  //Gyro Connection
  if (!gyro.begin_I2C()) {
    while (1) { delay(10); }
  }
  gyro.enableReport(SH2_GAME_ROTATION_VECTOR);

  //BT Connection
  bluetooth.begin("Skycrane");
  AlfredoConnect.begin(bluetooth);
}

void loop() {
  long time = millis();

  //We don't allow any code to progress past this point to prevent the robot from going crazy
  if (bluetooth.isClosed() | AlfredoConnect.getGamepadCount() < 1) {
    right_drive.set(0.0);
    left_drive.set(0.0);
    intake.set(0.0);
    right_shoot.set(0.0);
    left_shoot.set(0.0);   
    RSL::setState(RSL_DISABLED);
    return;
  } else {
    RSL::setState(RSL_ENABLED);
  }

  joy_throttle = AlfredoConnect.getAxis(0, 1);          // Collect raw joystick axis info for upcoming PID steering calcs
  joy_steer = AlfredoConnect.getAxis(0, 5);
  
  //PID / Steering Logic
  //
  if (gyro.getSensorEvent(&quat)) {
    quaternionToEuler(&quat, &rot);                     // Update our rot structure with the latest gyro information
    if (last_yaw > 170.0 && rot.yaw < -170.0){
      rotations++;                                      // Keep track of the # of rotations the bot has made
    }
    else if (last_yaw < -170.0 && rot.yaw > 170.0){
      rotations--;                                   
    }
    PID_input = rot.yaw + (double) rotations * 360.0;   // Basically makes an infinite numberline for our PID loop to work with
    last_yaw = rot.yaw;
  }
  

  if (PID_steering && abs(joy_steer) > 0.1){
    desired_yaw += (double)joy_steer * (time - last_time) * (drive_degrees_per_second / 1000.0);
    desired_yaw = wrap_yaw(desired_yaw);                // Desired yaw is in global gyro coordinates / -179.9 to 179.9
  }
  else{
    desired_yaw = rot.yaw;                              // Prevents robot from going to space when switching from normal to PID steering lol
  }
  
  PID_setpoint = desired_yaw + (double) rotations * 360.0;  // Converts our global desired yaw to a local infinite numberline using rot count
  
  // Drive control Logic
  //
  if (PID_steering){
    steer_loop.Compute();
    //todo - reverse
    drivetrain.arcadeDrive(joy_throttle, PID_output);
  }
  else{
    if (reverse_drive){
      drivetrain.arcadeDrive(-joy_throttle, joy_steer);
    }
    else{
      drivetrain.arcadeDrive(joy_throttle, joy_steer);
    }
  }
  
  // Servo Logic
  //
  int new_arm = AlfredoConnect.buttonHeld(0,4) - AlfredoConnect.buttonHeld(0, 2);
  switch (new_arm - joy_arm){
    case 1:
      arm_position++;
      break;
    case -1:
      arm_position--;
      break;
  }
  joy_arm = new_arm;
  set_arm_servo();
  
  int new_defense = AlfredoConnect.buttonHeld(0,5) - AlfredoConnect.buttonHeld(0, 3);
  switch (new_defense - joy_defense){
    case 1:
      defense_position++;
      break;
    case -1:
      defense_position--;
      break;
  }
  joy_defense = new_defense;
  set_defense_servo();

  //Intake Controls
  //
  if (AlfredoConnect.buttonHeld(0, 1)) {
    intake.set(1.0);
  } else {
    intake.set(0.0);
  }

  //Shooter Controls
  if (AlfredoConnect.buttonHeld(0, 0)) {
    left_shoot.set(-1.0);
    right_shoot.set(-1.0);
    if (time - shooter_time > 2750) {
      intake.set(1.0);
    }
  } else {
    shooter_time = time;
    left_shoot.set(0.0);
    right_shoot.set(0.0);
  }

  if (time - debug_timer > 100) {
    debug_timer = time;
  }

  last_time = time;
  AlfredoConnect.update();  //end of loop
  RSL::update();
}

void set_arm_servo() {
  // bluetooth.println(arm_position);
  if (arm_position == 0) {
    arm_servo.write(arm_intake);
  } else if (arm_position == 1) {
    arm_servo.write(arm_closeshot);
  } else if (arm_position == 2) {
    arm_servo.write(arm_farshot);
  }
}

void set_defense_servo() {
  // bluetooth.println(defense_position);
  if (defense_position == 0) {
    defense_servo.write(defense_down);
  } else if (defense_position == 1) {
    defense_servo.write(defense_chival);
  } else if (defense_position == 2) {
    defense_servo.write(defense_up);
  }
}

void quaternionToEuler(sh2_SensorValue_t* quat, euler_t* ypr) {
  float qr = quat->un.gameRotationVector.real;
  float qi = quat->un.gameRotationVector.i;
  float qj = quat->un.gameRotationVector.j;
  float qk = quat->un.gameRotationVector.k;
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = (double)atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * -RAD_TO_DEG;
  ypr->pitch = (double)asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)) * RAD_TO_DEG;
  ypr->roll = (double)atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) * RAD_TO_DEG;
}

double wrap_yaw(double x) {
  if (x > 180.0) {
    return x - 360.0;
  } else if (x < -180.0) {
    return x + 360.0;
  } else {
    return x;
  }
}