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

//Encoder Setup
//
#define left_encoder 2
#define right_encoder 33
bool left_state;
bool right_state;
int left_count;
int right_count;
int last_left_count;
int last_right_count;
float left_rpm;
float right_rpm;
int encoder_update_period = 20;  // ms
long last_encoder_update = 0;

//Limit Setup
//
#define intake_limit 34
bool ball_ready = false;

// NoU Setup
//
NoU_Motor left_drive(2);
NoU_Motor right_drive(1);
NoU_Motor left_shoot(4);
NoU_Motor intake(5);
NoU_Motor right_shoot(3);
NoU_Servo arm_servo(1);
NoU_Servo defense_servo(2);

//Servo positions
//
#define arm_down 155
#define arm_highshot 20
#define arm_lowshot 120
#define defense_up 180
#define defense_down 90


//Controls Setup
//
float throttle = 0.0;
int arm_position = 1;  // 0, 1, 2 == intake, far shot, close shot
bool arm_up_button = false;
bool arm_down_button = false;
int defense_position = 1;  // 0, 1, 2 == down, chival, up
long shooter_time = 0;

double start_yaw; // Direction we are facing on startup
double start_pitch; // Pitch on startup
double set_yaw; // Desired facing direction, as modified by the joystick
double steer = 0.0; // PID steering output to be fed into drive system
bool drive_direction = true; // true = shooter forward
PID steer_loop(&rot.yaw, &steer, &set_yaw, 1.0, 0.1, 0.5, DIRECT); //P 1.0, I 0.1, D 0.5
// current orientation is stored in rot struct

//Code State
//
bool first_loop = true;
long last_time = millis();



BluetoothSerial bluetooth;

void setup() {
  pinMode(left_encoder, INPUT);
  pinMode(right_encoder, INPUT);
  pinMode(intake_limit, INPUT);

  if (!gyro.begin_I2C()) {
    while (1) { delay(10); }
  }
  gyro.enableReport(SH2_GAME_ROTATION_VECTOR);
  steer_loop.SetOutputLimits(-1.0, 1.0);
  bluetooth.begin("Skycrane");
  AlfredoConnect.begin(bluetooth);
  RSL::initialize();
  RSL::setState(RSL_DISABLED);
}

void loop() {
  long time = millis();

  // update rotation from gyro
  //
  if (gyro.getSensorEvent(&quat)) {
    quaternionToEuler(&quat, &rot);
  }

  steer_loop.Compute();

  // update encoder data
  //
  bool left = digitalRead(left_encoder);
  bool right = digitalRead(right_encoder);
  if (left != left_state) {
    left_state = left;
    left_count++;
  }
  if (right != right_state) {
    right_state = right;
    right_count++;
  }

  if (time - encoder_update_period > last_encoder_update) {
    left_rpm = (float)(left_count - last_left_count) / encoder_update_period;
    right_rpm = (float)(right_count - last_right_count) / encoder_update_period;
    last_left_count = left_count;
    last_right_count = right_count;
    last_encoder_update = time;
  }

  //Get controls from alfredoconnect
  //
  if (AlfredoConnect.getGamepadCount() > 0) {
    throttle = -AlfredoConnect.getAxis(0, 1);
    long delta = (float)time - last_time;
    set_yaw = fmod((set_yaw + (double)AlfredoConnect.getAxis(0, 5) * delta), 360.0);
    

    //Main Arm Controls
    bool joystick_arm_up = AlfredoConnect.buttonHeld(0, 4);
    bool joystick_arm_down = AlfredoConnect.buttonHeld(0, 2);

    if (joystick_arm_up && !arm_up_button && arm_position < 2) {
      arm_position++;
      set_arm_servo();
    } else if (joystick_arm_down && !arm_down_button && arm_position > 0) {
      arm_position--;
      set_arm_servo();
    }
    arm_up_button = joystick_arm_up;
    arm_down_button = joystick_arm_down;

    //Intake Controls
    if (AlfredoConnect.buttonHeld(0, 1) && !ball_ready) {
      intake.set(1.0);
    } else {
      intake.set(0.0);
    }

    //Shooter Controls
  if (AlfredoConnect.buttonHeld(0, 0)){
      left_shoot.set(-1.0);
      right_shoot.set(-1.0);
    if(time-shooter_time > 2750){
      intake.set(1.0);      
    }          
  } else {
      shooter_time = time;
      left_shoot.set(0.0);
      right_shoot.set(0.0);    
  }
    RSL::setState(RSL_ENABLED);
    if (first_loop) {
      first_loop = false;
      bluetooth.println("Skycrane is online!");
      start_yaw = rot.yaw;
      start_pitch = rot.pitch;
      set_yaw = start_yaw;
      steer_loop.SetMode(AUTOMATIC); // Located down here so that we have actual gyro data on startup
    }
  } else {
    RSL::setState(RSL_DISABLED);
  }

  if (time % 1000 == 0){
    bluetooth.println(rot.yaw, set_yaw);
  }

  last_time = time;
  AlfredoConnect.update();  //end of loop
  RSL::update();
}

void set_arm_servo() {
  // bluetooth.println(arm_position);
  if (arm_position == 0) {
    arm_servo.write(arm_down);
  } else if (arm_position == 1) {
    arm_servo.write(arm_lowshot);
  } else if (arm_position == 2) {
    arm_servo.write(arm_highshot);
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

  ypr->yaw = (double)atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * RAD_TO_DEG;
  ypr->pitch = (double)asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)) * RAD_TO_DEG;
  ypr->roll = (double)atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) * RAD_TO_DEG;
}
