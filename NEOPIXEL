#include <Servo.h>
#include <IRremote.h>
#include <Adafruit_NeoPixel.h>

// ===== IR CONFIG =====
const int IR_RECEIVE_PIN = 9;
unsigned long lastPressTime = 0;
const int RELEASE_TIMEOUT = 200;
int mode = 1;

// ===== IR HEX CODES =====
#define BTN_UP        0xB946FF00
#define BTN_RIGHT     0xBC43FF00
#define BTN_DOWN      0xEA15FF00
#define BTN_LEFT      0xBB44FF00

#define BTN_MODE1     0xF30CFF00
#define BTN_MODE2     0xE718FF00
#define BTN_MODE3     0xA15EFF00
#define BTN_MODE4     0xF708FF00

#define BTN_SPEED_DOWN 0xBD42FF00
#define BTN_SPEED_UP   0xAD52FF00

// ===== MOTOR PINS =====
int PWMA = 5, AIN1 = 7;
int PWMB = 6, BIN1 = 8;
int STBY = 3;

// ===== SPEED SYSTEM =====
int speedValue = 120;
const int SPEED_STEP = 20;
const int MAX_SPEED = 255;
const int MIN_SPEED = 60;

// ===== ULTRASONIC =====
Servo myservo;
const int Trig = 13;
const int Echo = 12;

// ===== THRESHOLDS =====
const int THRESH = 350;   // black/white cutoff

// ===== NeoPixel LED =====
#define LED_PIN 4
#define NUM_LEDS 1
Adafruit_NeoPixel led(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

void colorMode1() { setColor(0, 0, 255); }
void colorMode2() { setColor(0, 255, 0); }
void colorMode3() { setColor(255, 0, 0); }
void colorMode4() { setColor(150, 0, 255); }
void colorLostLine() { setColor(255, 180, 0); }

// =============================================================
//                    IR RECEIVING
// =============================================================
uint32_t readIR() {
  if (IrReceiver.decode()) {
    uint32_t value = IrReceiver.decodedIRData.decodedRawData;
    IrReceiver.resume();
    lastPressTime = millis();
    return value;
  }
  return 0;
}

// =============================================================
//                    SMART DELAY
// =============================================================
void smartDelay(int ms) {
  for (int i = 0; i < ms; i += 5) {
    readIR();
    delay(5);
  }
}

// =============================================================
//                         SETUP
// =============================================================
void setup() {
  Serial.begin(9600);

  led.begin();
  led.show();

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);

  myservo.attach(10);

  colorMode1();
  Serial.println("READY.");
}

// =============================================================
//                         LOOP
// =============================================================
void loop() {

  uint32_t code = readIR();

  if (code == BTN_MODE1) { mode = 1; colorMode1(); }
  if (code == BTN_MODE2) {
    if (mode == 2) { mode = 1; colorMode1(); }
    else { mode = 2; colorMode2(); }
  }
  if (code == BTN_MODE3) { mode = 3; colorMode3(); }
  if (code == BTN_MODE4) { mode = 4; colorMode4(); }

  switch (mode) {
    case 1: modeManual(code); break;
    case 2: modeLineStrong(); break;
    case 3: modeUltrasonic(); break;
    case 4: modeVictoryLap(); break;
  }
}

// =============================================================
//              MODE 1 — TRUE HOLD MANUAL MODE
// =============================================================
void modeManual(uint32_t code) {

  if (code == BTN_SPEED_UP)  
    speedValue = constrain(speedValue + SPEED_STEP, MIN_SPEED, MAX_SPEED);

  if (code == BTN_SPEED_DOWN)  
    speedValue = constrain(speedValue - SPEED_STEP, MIN_SPEED, MAX_SPEED);

  if      (code == BTN_UP)    forward(speedValue);
  else if (code == BTN_DOWN)  backward(speedValue);
  else if (code == BTN_LEFT)  left(speedValue);
  else if (code == BTN_RIGHT) right(speedValue);
  else if (millis() - lastPressTime > RELEASE_TIMEOUT) stopMotors();
}

// =============================================================
//     MODE 2 — STRONG CENTERING PID WITH 5-SEC SPIN SEARCH
// =============================================================
float Kp = 22;
float Kd = 120;
float Ki = 0.01;

float lastError = 0;
float integral = 0;

void modeLineStrong() {

  int L = analogRead(A0);
  int M = analogRead(A1);
  int R = analogRead(A2);

  // =====================================================
  //              LOST LINE — 5-SECOND SPIN
  // =====================================================
  if (L < 100 && M < 100 && R < 100) {

    colorLostLine();

    unsigned long startTime = millis();

    // Spin right in place for 5 seconds MAX
    while (millis() - startTime < 5000) {

      // Spin robot: left wheel forward, right wheel backward
      driveMotors(150, -150);

      smartDelay(50);

      // Re-read sensors
      L = analogRead(A0);
      M = analogRead(A1);
      R = analogRead(A2);

      // FOUND LINE → stop immediately
      if (L > THRESH || M > THRESH || R > THRESH) {
        stopMotors();
        return;
      }
    }

    // After full spin attempt → stop and give up
    stopMotors();
    return;
  }

  colorMode2();

  // =====================================================
  //                STRONG CORRECTION RULES
  // =====================================================
  if (L > THRESH && M < THRESH && R < THRESH) {
    driveMotors(120, 200);  
    return;
  }

  if (R > THRESH && M < THRESH && L < THRESH) {
    driveMotors(200, 120);  
    return;
  }

  // Full black = junction or error → stop
  if (L > THRESH && M > THRESH && R > THRESH) {
    stopMotors();
    return;
  }

  // =====================================================
  //                        PID
  // =====================================================
  float position = (R * 1000.0 + L * -1000.0) / (L + M + R);
  float error = position / 1000.0;

  integral += error;
  float derivative = error - lastError;
  lastError = error;

  float output = Kp * error + Kd * derivative + Ki * integral;

  int baseSpeed = 150;
  int leftPWM  = baseSpeed - output;
  int rightPWM = baseSpeed + output;

  driveMotors(constrain(leftPWM, 0, 255), constrain(rightPWM, 0, 255));
}

// =============================================================
//      MODE 3 — OBSTACLE AVOIDANCE
// =============================================================
float DistanceMeter() {
  digitalWrite(Trig, LOW); delayMicroseconds(2);
  digitalWrite(Trig, HIGH); delayMicroseconds(10);
  digitalWrite(Trig, LOW);

  long d = pulseIn(Echo, HIGH, 25000);
  if (d == 0) return 0;

  float cm = (d * 0.0343) / 2;
  return (cm > 300) ? 0 : cm;
}

void modeUltrasonic() {

  myservo.write(90);
  smartDelay(150);

  float front = DistanceMeter();

  if (front > 25 || front == 0) {
    forward(120);
    return;
  }

  stopMotors();
  smartDelay(120);

  myservo.write(150);
  smartDelay(200);
  float leftDist = DistanceMeter();

  if (leftDist > 25) {
    left(140);
    smartDelay(400);
    return;
  }

  myservo.write(30);
  smartDelay(200);
  float rightDist = DistanceMeter();

  if (rightDist > 25) {
    right(140);
    smartDelay(400);
    return;
  }

  backward(140);
  smartDelay(350);

  if (leftDist > rightDist) left(160);
  else right(160);

  smartDelay(600);
}

// =============================================================
//                MODE 4 — VICTORY LAP
// =============================================================
void modeVictoryLap() {
  forward(150); smartDelay(6000);
  left(130); smartDelay(700);

  forward(150); smartDelay(3500);
  left(130); smartDelay(700);

  forward(150); smartDelay(6000);
  left(130); smartDelay(700);

  forward(150); smartDelay(3500);
  left(130); smartDelay(700);

  stopMotors();
}

// =============================================================
//                MOTOR FUNCTIONS
// =============================================================
void driveMotors(int leftPWM, int rightPWM) {

  if (leftPWM >= 0) { digitalWrite(AIN1, HIGH); analogWrite(PWMA, leftPWM); }
  else              { digitalWrite(AIN1, LOW);  analogWrite(PWMA, -leftPWM); }

  if (rightPWM >= 0) { digitalWrite(BIN1, HIGH); analogWrite(PWMB, rightPWM); }
  else               { digitalWrite(BIN1, LOW);  analogWrite(PWMB, -rightPWM); }
}

void forward(int s){
  digitalWrite(AIN1, HIGH); analogWrite(PWMA, s);
  digitalWrite(BIN1, HIGH); analogWrite(PWMB, s);
}

void backward(int s){
  digitalWrite(AIN1, LOW); analogWrite(PWMA, s);
  digitalWrite(BIN1, LOW); analogWrite(PWMB, s);
}

void left(int s){
  digitalWrite(AIN1, HIGH); analogWrite(PWMA, s);
  digitalWrite(BIN1, LOW);  analogWrite(PWMB, s);
}

void right(int s){
  digitalWrite(AIN1, LOW);  analogWrite(PWMA, s);
  digitalWrite(BIN1, HIGH); analogWrite(PWMB, s);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}
