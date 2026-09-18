#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <MPU6050_light.h>

// ========== MOTOR PINS ==========
#define LPWM_L 14
#define RPWM_L 12
#define LPWM_R 2
#define RPWM_R 4

#define PWMFreq 1000
#define PWMResolution 8

// ========== ENCODER PINS ==========
#define ENCODER_LEFT_A   32
#define ENCODER_LEFT_B   33
#define ENCODER_RIGHT_A  16
#define ENCODER_RIGHT_B  17

volatile int leftCount = 0;
volatile int rightCount = 0;
int wavepoint = 1;
float yaw = 0;
float setpoint = 90;

// ========== MPU6050 ==========
MPU6050 mpu(Wire);

// ========== PD Controller ==========
float Kp = 2.0;
float Kd = 0.5;
float lastError = 0;

// ========== WiFi ==========
const char* ssid = "OPPO A55";
const char* password = "u49u56rb";
AsyncWebServer server(80);

// ========== ENCODER ISR ==========
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

// ========== MOTOR CONTROL ==========
void forward(int speed) {
  analogWrite(LPWM_L, speed);
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_R, speed);
  analogWrite(RPWM_R, 0);
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


// ========== SETUP ==========
void setup() {
  Serial.begin(115200);

  // Motor setup
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

  // MPU6050 setup
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0); // Stop if MPU not found
  Serial.println(F("Calculating MPU offsets..."));
  delay(1000);
  mpu.calcOffsets();
  Serial.println("MPU Calibration Done!");
  delay(5000);
  // WiFi setup
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  // Webpage route
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", R"rawliteral(
      <!DOCTYPE html>
      <html>
      <head>
        <title>ESP32 Telemetry</title>
        <meta charset="UTF-8">
        <style>
          body { font-family: Arial; text-align: center; padding: 30px; }
          h2 { font-size: 24px; }
          .data-box { font-size: 20px; margin: 15px; }
        </style>
      </head>
      <body>
        <h2>Planet</h2>
        <div class="data-box">Left Encoder: <span id="leftCount">--</span></div>
        <div class="data-box">Right Encoder: <span id="rightCount">--</span></div>
        <div class="data-box">Yaw (Z): <span id="yaw">--</span>°</div>

        <script>
          setInterval(() => {
            fetch("/data")
              .then(response => response.json())
              .then(data => {
                document.getElementById("leftCount").textContent = data.left;
                document.getElementById("rightCount").textContent = data.right;
                document.getElementById("yaw").textContent = data.yaw.toFixed(2);
              });
          }, 500);
        </script>
      </body>
      </html>
    )rawliteral");
  });

  // JSON telemetry endpoint
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{\"left\":" + String(leftCount) +
                  ",\"right\":" + String(rightCount) +
                  ",\"yaw\":" + String(yaw) + "}";
    request->send(200, "application/json", json);
  });

  server.begin();
  delay(5000);
}

// ========== MAIN LOOP ==========
void loop() {
  mpu.update();
  yaw = mpu.getAngleZ();  
  int lCount, rCount;
  noInterrupts();
  lCount = leftCount;
  rCount = rightCount;
  interrupts();

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
      left(30);
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
      left(30);
    }
    else
    {
    stop();
    wavepoint=7;
    // setpoint=180;
    }
    break;

    case 7:
    if (lCount < 850) {
    forward(50);
  } else {
    stop();
    wavepoint=8;
    setpoint=276;
  }

  break;
  }

  delay(200); 

  // delay(200);
}
