#include <Alfredo_NoU2.h>
#include <AlfredoConnect.h>
#include <BluetoothSerial.h>
#include <Adafruit_BNO08x.h>

//Gyro Setup
Adafruit_BNO08x gyro(-1); // Might need the reset pin to be 5
sh2_SensorValue_t quat; //https://github.com/adafruit/Adafruit_BNO08x/blob/67b91b809da04a08fccb8793770343f872daaf43/src/sh2_SensorValue.h#L186
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} rot;

//Encoder Setup
#define left_encoder D2;
#define right_encoder D33;
bool left_state;
bool right_stat;
int left_count;
int right_count;
int last_left_count;
int last_right_count;
float left_rpm;
float right_rpm;
int encoder_update_period = 20; // ms
long last_encoder_update = 0;

//Limit Setup
#define intake_limit D34;
bool ball_ready;

// NoU Setup
NoU_Motor left_drive(2);
NoU_Motor right_drive(1);
NoU_Motor left_shoot(4);
NoU_Motor right_shoot(3);
NoU_Servo arm_servo(1);
NoU_Servo defense_servo(2);
BluetoothSerial bluetooth;

//Servo positions
#define arm_down 180;
#define arm_highshot 30;
#define arm_lowshot 160;
#define defense_up 180;
#define defense_down 90;

void setup() {
  pinMode(left_encoder, INPUT);
  pinMode(right_encoder, INPUT);
  pinMode(intake_limit, INPUT);
  
  if (!gyro.begin_I2C()){
    while (1) {delay(10);}
  }
  gyro.enableReport(SH2_GAME_ROTATION_VECTOR);
  bluetooth.begin("Skycrane");
  AlfredoConnect.begin(bluetooth);
  bluetooth.println("Skycrane is online!");
}

void loop() {
  long time = millis();
  
  // update rotation from gyro
  if (gyro.getSensorEvent(&quat)){
    quaternionToEuler(&quat, &rot);
  }
  
  // update encoder data
  bool left = digitalRead(left_encoder);
  bool right = digitalRead(right_encoder);
  if (left != left_state){
    left_state = left;
    left_count++;
  }
  if (right != right_state){
    right_state = right;
    right_count++;
  }
  
  
  
  AlfredoConnect.update(); //end of loop
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

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * RAD_TO_DEG;
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)) * RAD_TO_DEG;
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) * RAD_TO_DEG;
}
