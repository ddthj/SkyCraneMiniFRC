#include <Alfredo_NoU2.h>
#include <AlfredoConnect.h>
#include <BluetoothSerial.h>
#include <Adafruit_BNO08x.h>
#include <PID_v1.h>  // https://github.com/br3ttb/Arduino-PID-Library/blob/master/examples/PID_Basic/PID_Basic.ino

//Gyro Setup
//
Adafruit_BNO08x gyro(-1);
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
#define arm_intake 152
#define arm_closeshot 20
#define arm_farshot 118

#define defense_up 40
#define defense_chival 90
#define defense_down 180


//Controls Setup
//
float joy_throttle = 0.0;     // raw joystick throttle input
float joy_steer = 0.0;        // raw joystick steering input
int joy_arm = 0;              // button state of joystick arm control
int joy_intake = 0;          // button state of joystick defense arm control
bool joy_reverse = false;     // button state of joystick reverse-drive control
bool joy_PID = false;

bool PID_steering = false;    // determins if the joystick is steering or if the PID loop is steering
bool reverse_drive = true;  // true = shooter forward
int arm_position = 2;         // 0, 1, 2 == intake, far shot, close shot
int defense_position = 2;     // 0, 1, 2 == down, chival, up
long shooter_time = 0;        // Timer for shooter flywheel spoolup

//Starting yaw/pitch is always 0/0
#define drive_degrees_per_second 90.0
double last_yaw = 0.0;     // last yaw reading, used to keep track of cumulative yaw
int rotations = 0;         // keeps track of # of rotations the bot has made in either direction
double PID_input = 0.0;    // cumulative yaw. wraps around when passing 180° so that the PID loop doesn't get confused
double PID_setpoint = 0.0; // desired_yaw + cum_yaw
double PID_output = 0.0;   // Result of the PID computation, used to steer
double desired_yaw = 0.0;  // Desired facing direction, as modified by the joystick
bool on_target = false;

PID steer_loop(&PID_input, &PID_output, &PID_setpoint, 0.08, 0.17, 0.005, DIRECT); //P 0.11, I 0.15, D 0.005

bool auto_aiming = false;
double auto_aim_target = 0.0;

int autonomous = 0;      // 1-5 for each defense (lowbar included)
int autonomous_step = 0; // 0 = lower intake, 1 = drive until pitchdown, 2 = drive until level, 3 = auto-aim and set arm, 4 = shoot, unknown = disable auto
bool autonomous_low = false; // Keeps track of whether we are going through a port or low bar and need the arm low
long autonomous_timer = 0;
float autonomous_throttle = 0.0;
bool autonomous_shoot = false;

void autonomous_reset(){
  autonomous = 0;
  autonomous_step = 0;
  autonomous_low = false;
  autonomous_throttle = 0.0;
  autonomous_shoot = false;
}

int auto_delay(float seconds){
  return (int)(seconds * 1000);
}

void set_auto_aim(double target){
  PID_steering = true;
  auto_aiming = true;
  auto_aim_target = target;
}

//Code State
//
long last_time = millis();
long debug_timer = 0;

BluetoothSerial bluetooth;

void setup() {
  //Misc setup
  steer_loop.SetOutputLimits(-1.0, 1.0);
  steer_loop.SetSampleTime(20);
  steer_loop.SetMode(AUTOMATIC);
  drivetrain.setInputExponent(0.55);
  
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
  while (AlfredoConnect.getGamepadCount() < 1){
    right_drive.set(0.0);
    left_drive.set(0.0);
    intake.set(0.0);
    right_shoot.set(0.0);
    left_shoot.set(0.0);   
    RSL::setState(RSL_DISABLED);
    AlfredoConnect.update();  //end of loop
    RSL::update();
  }
  RSL::setState(RSL_ENABLED);

  //Autonomous Mode
  if (autonomous && time > autonomous_timer){
    if (autonomous_step == 0){
      // Lower arm! The lowbar and port need the intake position, everything else needs chival
      // autonomous_low is set by pressing the key below the default autonomous key for that defense position.
      if (autonomous_low){
        arm_position = 0;
      }
      else{
        arm_position = 1;
      }
      autonomous_timer = time + auto_delay(1.0);
      autonomous_step++;
    }
    else if (autonomous_step == 1){
      // Drive the robot full speed until we have pitched down at least 5°
      autonomous_throttle = 0.75;
      if (rot.roll < -5.0){
        autonomous_step++;
      }
    }
    else if (autonomous_step == 2){
      autonomous_throttle = 0.25;
      if (abs(rot.roll < 0.2)){
        autonomous_step++;
        if (autonomous == 1){
          autonomous_timer = time + auto_delay(2.0);
        }
      }
    }
    else if (autonomous_step == 3){
      autonomous_throttle = 0.0;
      arm_position = 1;
      if (autonomous == 1){
        set_auto_aim(70.0);
      } else if (autonomous == 2){
        set_auto_aim(40.0);
      } else if (autonomous == 3){
        set_auto_aim(13.0);
      } else if (autonomous == 4){
        set_auto_aim(-13.0);
      } else if (autonomous == 5){
        set_auto_aim(-40.0);
      }
      autonomous_step++;
    }
    else if (autonomous_step == 4){
      if (!auto_aiming){
        autonomous_shoot = true;
        autonomous_step++;
        autonomous_timer = time + auto_delay(4.0);
      }
    }
    else{
      autonomous_reset();
    }
  }
  else{
    if (AlfredoConnect.keyHeld(Key::Q)){
      autonomous = 1;
    } else if (AlfredoConnect.keyHeld(Key::A)){
      autonomous = 1;
      autonomous_low = true;
    } else if (AlfredoConnect.keyHeld(Key::W)){
      autonomous = 2;
    } else if (AlfredoConnect.keyHeld(Key::S)){
      autonomous = 2;
      autonomous_low = true;
    } else if (AlfredoConnect.keyHeld(Key::E)){
      autonomous = 3;
    } else if (AlfredoConnect.keyHeld(Key::D)){
      autonomous = 3;
      autonomous_low = true;
    } else if (AlfredoConnect.keyHeld(Key::R)){
      autonomous = 4;
    } else if (AlfredoConnect.keyHeld(Key::F)){
      autonomous = 4;
      autonomous_low = true;
    } else if (AlfredoConnect.keyHeld(Key::T)){
      autonomous = 5;
    } else if (AlfredoConnect.keyHeld(Key::G)){
      autonomous = 5;
      autonomous_low = true;
    }
  }

  // Collect raw joystick axis info for upcoming PID steering calcs
  if (autonomous){
    joy_throttle = autonomous_throttle; // auto hijacks this lol
  }
  else{
    joy_throttle = AlfredoConnect.getAxis(0, 1);          
  }   
  joy_steer = AlfredoConnect.getAxis(0, 5);
  
  // Reverse direction button
  bool new_reverse = AlfredoConnect.buttonHeld(0, 10);
  if (new_reverse && new_reverse != joy_reverse){
    reverse_drive = !reverse_drive;
  }
  joy_reverse = new_reverse;
  
  //PID steering button
  //
  bool new_PID = AlfredoConnect.buttonHeld(0, 11);
  if (new_PID && new_PID != joy_PID){
    PID_steering = !PID_steering;
  }
  joy_PID = new_PID;
  
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

  if (PID_steering){
    // todo - might want to avoid this during auto?
    if (abs(joy_steer) > 0.1){
      desired_yaw += (double)joy_steer * (time - last_time) * (drive_degrees_per_second / 1000.0);
    }
  }
  else{
    desired_yaw = rot.yaw + (double) rotations * 360.0; // Prevents robot from going to space when switching from normal to PID steering lol
  }
  
  
  if (auto_aiming){
    desired_yaw = auto_aim_target + (double) rotations * 360.0;
    on_target = abs(PID_input - desired_yaw) < 1.0;
    if (on_target){
      bluetooth.println("Locked on!");
      auto_aiming = false;
      PID_steering = false;
    }
  }
  else{
    if (AlfredoConnect.keyHeld(Key::Digit1)){
      bluetooth.println("Defense 1 (lowbar) Auto-Aim...");
      set_auto_aim(70.0);
      arm_position = 1;
    }
    else if (AlfredoConnect.keyHeld(Key::Digit2)){
      bluetooth.println("Defense 2 Auto-Aim...");
      set_auto_aim(40.0);
      arm_position = 1;
    }
    else if (AlfredoConnect.keyHeld(Key::Digit3)){
      bluetooth.println("Defense 3 Auto-Aim...");
      set_auto_aim(13.0);
      arm_position = 1;
    }
    else if (AlfredoConnect.keyHeld(Key::Digit4)){
      bluetooth.println("Defense 4 Auto-Aim...");
      set_auto_aim(-13.0);
      arm_position = 1;
    }
    else if (AlfredoConnect.keyHeld(Key::Digit5)){
      bluetooth.println("Defense 5 Auto-Aim...");
      set_auto_aim(-40.0);
      arm_position = 1;
    }
  }

  PID_setpoint = desired_yaw;

  // Drive control Logic
  //
  if (PID_steering){
    steer_loop.Compute();
    if (reverse_drive){
      drivetrain.arcadeDrive(-joy_throttle, PID_output);
    }
    else {
      drivetrain.arcadeDrive(joy_throttle, PID_output); 
    }
  }
  else{
    if (reverse_drive){
      drivetrain.arcadeDrive(-joy_throttle, joy_steer);
    }
    else {
      drivetrain.arcadeDrive(joy_throttle, joy_steer);
    }
  }
  
  // Servo Logic
  //
  int new_arm = AlfredoConnect.buttonHeld(0,4) - AlfredoConnect.buttonHeld(0, 2);
  if (new_arm != joy_arm){
    if (new_arm > 0 && arm_position < 2){
      arm_position++;
    }
    else if (new_arm < 0 && arm_position > 0){
      arm_position--;
    } 
  }
  joy_arm = new_arm;
  set_arm_servo();
  
  /*
  int new_defense = AlfredoConnect.buttonHeld(0,5) - AlfredoConnect.buttonHeld(0, 3);
  if (new_defense != joy_defense){
    if (new_defense > 0 && defense_position < 2){
      defense_position++;
    }
    else if (new_defense < 0 && defense_position > 0){
      defense_position--;
    }
  }
  joy_defense = new_defense;
  
  */
  defense_position = arm_position; // Simplifes controls
  set_defense_servo();

  //Intake Controls
  //
  intake.set((float) AlfredoConnect.buttonHeld(0,3) - AlfredoConnect.buttonHeld(0, 5));

  //Shooter Controls
  if (AlfredoConnect.buttonHeld(0, 0) | (autonomous && autonomous_shoot)) {
    left_shoot.set(-1.0);
    right_shoot.set(-1.0);
    if (time - shooter_time > 2500) {
      intake.set(1.0);
    }
  } else {
    shooter_time = time;
    left_shoot.set(0.0);
    right_shoot.set(0.0);
  }

  if (time - debug_timer > 100) {
    debug_timer = time;
    if (autonomous){
      bluetooth.println(autonomous_step);
    }
    //bluetooth.println(rot.roll);
    //bluetooth.print("  ");
    //bluetooth.print(PID_setpoint);
    //bluetooth.print("  ");
    //bluetooth.println(PID_output);
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
    arm_servo.write(arm_farshot);
  } else if (arm_position == 2) {
    arm_servo.write(arm_closeshot);
  }
}

void set_defense_servo() {
  // bluetooth.println(defense_position);
  if (defense_position == 0) {
    defense_servo.write(defense_up);
  } else if (defense_position == 1) {
    defense_servo.write(defense_chival);
  } else if (defense_position == 2) {
    defense_servo.write(defense_down);
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