#include <Wire.h>
#include <PS4Controller.h>
#include <MPU6050_light.h>

// ================= MOTOR PINS =================
#define LPWM_L 14
#define RPWM_L 12
#define LPWM_R 2
#define RPWM_R 4

#define PWMFreq 1000
#define PWMResolution 8

// ================= ENCODER PINS =================
#define ENCODER_LEFT_A   32
#define ENCODER_LEFT_B   33
#define ENCODER_RIGHT_A  16
#define ENCODER_RIGHT_B  17

volatile int leftCount = 0;
volatile int rightCount = 0;
int wavepoint=1;
float yaw=0;
float setpoint=90;
// ================= MPU6050 =================
MPU6050 mpu(Wire);
unsigned long gyroTimer = 0;


// ======= PD Controller Variables =======
float Kp = 2.0;   // Proportional gain
float Kd = 0.5;   // Derivative gain

float lastError = 0;
unsigned long lastPIDTime = 0;

// ================= ENCODER ISR =================
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENCODER_LEFT_B) == HIGH)
    leftCount--;
  else
    leftCount++;
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENCODER_RIGHT_B) == HIGH)
    rightCount++;
  else
    rightCount--;
}

// ================= MOTOR FUNCTIONS =================
void forward(int speed) {
  analogWrite(LPWM_L, speed);
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_R, speed);
  analogWrite(RPWM_R, 0);
  Serial.println("Forward");
}

void backward(int speed) {
  analogWrite(LPWM_L, 0);
  analogWrite(RPWM_L, speed);
  analogWrite(LPWM_R, 0);
  analogWrite(RPWM_R, speed);
}

void left(int speed) {
  analogWrite(LPWM_L, 0);
  analogWrite(RPWM_L, speed);
  analogWrite(LPWM_R, speed);
  analogWrite(RPWM_R, 0);
}

void right(int speed) {
  analogWrite(LPWM_L, speed);
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_R, 0);
  analogWrite(RPWM_R, speed);
}

void stop() {
  analogWrite(LPWM_L, 0);
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_R, 0);
  analogWrite(RPWM_R, 0);
}

// ================= PS4 CALLBACK =================
void notify() {
  int x = PS4.RStickX();
  int y = PS4.RStickY();
  int speed = map(max(abs(x), abs(y)), 0, 128, 0, 255);

  if (abs(x) < 15 && abs(y) < 15) {
    stop();
    return;
  }

  if (PS4.R2()) forward(90);
  else if (PS4.L2()) backward(90);
  else if (PS4.L1()) left(90);
  else if (PS4.R1()) right(90);
  else if (y < -15) forward(speed);
  else if (y > 15) backward(speed);
  else if (x < -15) left(speed);
  else if (x > 15) right(speed);
  else stop();
}

void onConnect() {
  Serial.println("PS4 Connected");
}

void onDisConnect() {
  stop();
  Serial.println("PS4 Disconnected");
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  // Motor pin setup
  pinMode(LPWM_L, OUTPUT); pinMode(RPWM_L, OUTPUT);
  pinMode(LPWM_R, OUTPUT); pinMode(RPWM_R, OUTPUT);

  analogWriteResolution(LPWM_L, PWMResolution);
  analogWriteResolution(RPWM_L, PWMResolution);
  analogWriteResolution(LPWM_R, PWMResolution);
  analogWriteResolution(RPWM_R, PWMResolution);

  analogWriteFrequency(LPWM_L, PWMFreq);
  analogWriteFrequency(RPWM_L, PWMFreq);
  analogWriteFrequency(LPWM_R, PWMFreq);
  analogWriteFrequency(RPWM_R, PWMFreq);

  // Encoder setup
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, RISING);

  // PS4 setup
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin();
  Serial.println("Ready.");

  // MPU6050 setup
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0); // halt if MPU not found

  Serial.println(F("Calculating MPU6050 offsets..."));
  delay(1000);
  mpu.calcOffsets(); // auto-calibrate
  Serial.println("Done!");
}

// ================= LOOP =================
void loop() {
  // === Gyro Update ===
  mpu.update();

  if ((millis() - gyroTimer) > 10) { // print angles every 10ms
    // Serial.print("X: ");
    // Serial.print(mpu.getAngleX());
    // Serial.print("\tY: ");
    // Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");
    Serial.println(mpu.getAngleZ());
    gyroTimer = millis();
  }

  // === Encoder Feedback ===
  int lCount, rCount;
  noInterrupts();
  lCount = leftCount;
  rCount = rightCount;
  interrupts();

  Serial.print("Left Encoder: ");
  Serial.print(lCount);
  Serial.print(" | Right Encoder: ");
  Serial.println(rCount);

  // Example control using encoder
  switch(wavepoint)
  {
    case 1:
    if (lCount < 370) {
    forward(50);
  } else {
    stop();
    wavepoint=2;
    setpoint=73;
  }
    break;
    case 2:
    if(yaw<setpoint){

      float error = setpoint - yaw;
      float derivative = (error - lastError);

      float output = Kp * error + Kd * derivative;
      int pwmSpeed = constrain(output, 0, 70);  // Clamp speed to [0, 255]

      left(pwmSpeed); // Turn left with PD-controlled speed

      lastError = error;
      yaw=mpu.getAngleZ();
      left(30);
    }
    else
    {
      wavepoint=3;
      stop();
      // lCount=0;
      // rCount=0;
    }

    break;
    case 3:
    if (lCount < 550) {
    forward(50);
  } else {
    stop();
    wavepoint=4;
    setpoint=160;
  }
  break;

  case 4:
      if(yaw<setpoint){

      float error = setpoint - yaw;
      float derivative = (error - lastError);

      float output = Kp * error + Kd * derivative;
      int pwmSpeed = constrain(output, 0, 70);  // Clamp speed to [0, 255]

      left(pwmSpeed); // Turn left with PD-controlled speed

      lastError = error;
      yaw=mpu.getAngleZ();
      // left(30);
    }
    else
    {
      stop();
    wavepoint=5;
    }
    break;

    case 5:
    if (lCount < 690) {
    forward(50);
  } else {
    stop();
    wavepoint=6;
    setpoint=240;
  }
  break;

  case 6:
        if(yaw<setpoint){

      float error = setpoint - yaw;
      float derivative = (error - lastError);

      float output = Kp * error + Kd * derivative;
      int pwmSpeed = constrain(output, 0, 70);  // Clamp speed to [0, 255]

      left(pwmSpeed); // Turn left with PD-controlled speed

      lastError = error;
      yaw=mpu.getAngleZ();
      // left(30);
    }
    else
    {
    stop();
    wavepoint=7;
    // setpoint=180;
    }
    break;

    case 7:
    if (lCount < 820) {
    forward(50);
  } else {
    stop();
    wavepoint=8;
    setpoint=276;
  }

  break;
  }
  // if (lCount < 500) {
  //   forward(50);
  // } else {
  //   stop();
  // }

  delay(200); // adjust as needed
}