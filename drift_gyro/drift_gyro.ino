#include <EEPROM.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// ----- PIN DEFINITIONS -----
#define STEERING_INPUT_PIN 4
#define SERVO_OUTPUT_PIN A0
#define MODE_BUTTON_PIN 5
#define MODE_TOGGLE_PIN 6
#define BUZZER_PIN 7
#define LED_PIN 13
#define PID_TUNE_BUTTON_PIN 8

#define STEERING_NULL 1500
#define SERVO_MIN 900
#define SERVO_MAX 2100

// ----- EEPROM ADDRESSES -----
#define EEPROM_WEIGHT_ADDR 0
#define EEPROM_MODE_ADDR 1
#define EEPROM_ANGLE_KP_ADDR 10
#define EEPROM_ANGLE_KI_ADDR 14
#define EEPROM_ANGLE_KD_ADDR 18
#define EEPROM_RATE_KD_ADDR 22
#define EEPROM_ADAPT_KP_ADDR 26
#define EEPROM_ADAPT_KI_ADDR 30
#define EEPROM_ADAPT_KD_ADDR 34

// ----- CONSTANTS -----
#define SAMPLE 100
#define MAX_I_TERM 100
#define PID_TUNE_DELAY 500
#define LONG_PRESS_TIME 700

float angleKp, angleKi, angleKd;
float rateKd;
float adaptKp, adaptKi, adaptKd;

const float kpMin = 0.5, kpMax = 10.0, kpStep = 0.5;
const float kiMin = 0.05, kiMax = 1.0, kiStep = 0.05;
const float kdMin = 0.5, kdMax = 10.0, kdStep = 0.5;

// ----- USER CONTROL -----
#define NUM_WEIGHTS 11
float weightLevels[NUM_WEIGHTS] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
int currentWeightIndex;
float userWeight;

float G[3] = {0, 0, 0};  // G[2] is yaw rate (Z axis)

// ----- MODES -----
enum Mode { RATE, ANGLE, ADAPTIVE };
Mode currentMode = RATE;

float yawAngle = 0;
float yawRateFiltered = 0;
float iTerm_angle = 0;
float iTerm_adapt = 0;
float targetYaw = 0;

// ----- GYRO -----
MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;
float offsetGz = 0;
float gyroFilterAlpha = 0.1;

// ----- TIMING -----
unsigned long lastWeightButtonPress = 0;
unsigned long lastModeTogglePress = 0;
unsigned long timestamp;
unsigned long loop_timer;
unsigned long pidLastPress = 0;
unsigned long pidLastSwitch = 0;
bool pidWaiting = true;

// ----- PWM Input -----
volatile unsigned long timer[2];
volatile byte last_channel = 0;
volatile int input = STEERING_NULL;

int steer;
int c = 0;

enum PIDParam { PARAM_KP, PARAM_KI, PARAM_KD };
PIDParam currentParam = PARAM_KP;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(STEERING_INPUT_PIN, INPUT);
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_TOGGLE_PIN, INPUT_PULLUP);
  pinMode(PID_TUNE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20);

  DDRC |= B00000001;
  PORTC |= B00000001;
  delayMicroseconds(STEERING_NULL);
  PORTC &= ~B00000001;

  timestamp = millis();
  steer = STEERING_NULL;

  currentWeightIndex = EEPROM.read(EEPROM_WEIGHT_ADDR);
  if (currentWeightIndex < 0 || currentWeightIndex >= NUM_WEIGHTS) currentWeightIndex = 2;
  userWeight = weightLevels[currentWeightIndex];

  byte savedMode = EEPROM.read(EEPROM_MODE_ADDR);
  currentMode = (Mode)savedMode;

  EEPROM.get(EEPROM_ANGLE_KP_ADDR, angleKp);
  EEPROM.get(EEPROM_ANGLE_KI_ADDR, angleKi);
  EEPROM.get(EEPROM_ANGLE_KD_ADDR, angleKd);
  EEPROM.get(EEPROM_RATE_KD_ADDR, rateKd);
  EEPROM.get(EEPROM_ADAPT_KP_ADDR, adaptKp);
  EEPROM.get(EEPROM_ADAPT_KI_ADDR, adaptKi);
  EEPROM.get(EEPROM_ADAPT_KD_ADDR, adaptKd);

  if (angleKp < kpMin || angleKp > kpMax) angleKp = 3.5;
  if (angleKi < kiMin || angleKi > kiMax) angleKi = 0.1;
  if (angleKd < kdMin || angleKd > kdMax) angleKd = 1;
  if (rateKd < kdMin || rateKd > kdMax) rateKd = 1.5;
  if (adaptKp < kpMin || adaptKp > kpMax) adaptKp = 3.5;
  if (adaptKi < kiMin || adaptKi > kiMax) adaptKi = 0.1;
  if (adaptKd < kdMin || adaptKd > kdMax) adaptKd = 1;

  Wire.setClock(400000);
  Wire.begin();
  accelgyro.initialize();
  c = accelgyro.testConnection();

  calibrateGyro();
  indicateMode();

  pidLastSwitch = millis();
}

void loop() {
  loop_timer = millis() + 20;

  while (!c) c = accelgyro.testConnection();

  if (digitalRead(MODE_BUTTON_PIN) == LOW && millis() - lastWeightButtonPress > 125) {
    lastWeightButtonPress = millis();
    currentWeightIndex = (currentWeightIndex + 1) % NUM_WEIGHTS;
    userWeight = weightLevels[currentWeightIndex];
    EEPROM.update(EEPROM_WEIGHT_ADDR, currentWeightIndex);
    signalWeightLevel();
  }

  if (digitalRead(MODE_TOGGLE_PIN) == LOW) {
    unsigned long pressStart = millis();
    while (digitalRead(MODE_TOGGLE_PIN) == LOW);
    unsigned long pressDuration = millis() - pressStart;

    if (pressDuration > LONG_PRESS_TIME) {
      currentMode = ADAPTIVE;
      tone(BUZZER_PIN, 500, 400);
    } else {
      currentMode = (currentMode == RATE) ? ANGLE : RATE;
      tone(BUZZER_PIN, 1000, 150);
    }

    EEPROM.update(EEPROM_MODE_ADDR, (byte)currentMode);
    yawAngle = 0;
    iTerm_angle = 0;
    iTerm_adapt = 0;
    targetYaw = 0;
    indicateMode();
  }

  if (millis() - pidLastSwitch > PID_TUNE_DELAY) pidWaiting = false;
  if (!pidWaiting) handlePIDTuning();

  callimu();

  float control = 0;

  if (currentMode == ANGLE || currentMode == ADAPTIVE) {
    yawAngle += G[2] * 0.02;

    if (abs(input - STEERING_NULL) < 20) {
      yawAngle = 0;
      iTerm_angle = 0;
      iTerm_adapt = 0;
      if (currentMode == ADAPTIVE) targetYaw = 0;
    }

    yawAngle = constrain(yawAngle, -45.0, 45.0);
  }

  if (currentMode == RATE) {
    control = rateKd * G[2];
  } else if (currentMode == ANGLE) {
    float error = -yawAngle;
    iTerm_angle += angleKi * error;
    iTerm_angle = constrain(iTerm_angle, -MAX_I_TERM, MAX_I_TERM);
    control = -(angleKp * error + iTerm_angle - (G[2] * angleKd));
  } else if (currentMode == ADAPTIVE) {
    float steerInput = input - STEERING_NULL;
    targetYaw = constrain(steerInput * 0.05, -30, 30);
    float error = targetYaw - yawAngle;
    iTerm_adapt += adaptKi * error;
    iTerm_adapt = constrain(iTerm_adapt, -MAX_I_TERM, MAX_I_TERM);
    control = -(adaptKp * error + iTerm_adapt - (G[2] * adaptKd));
  }

  steer = input + (1.0 - userWeight) * control;
  steer = constrain(steer, SERVO_MIN, SERVO_MAX);

  motorWrite();
  while (millis() < loop_timer);
}

// ---------------- PID Tuning ------------------

void handlePIDTuning() {
  static bool lastState = HIGH;
  static unsigned long pressTime = 0;

  bool currentState = digitalRead(PID_TUNE_BUTTON_PIN);
  if (lastState == HIGH && currentState == LOW) pressTime = millis();

  if (lastState == LOW && currentState == HIGH) {
    unsigned long duration = millis() - pressTime;
    if (duration > LONG_PRESS_TIME) {
      currentParam = (PIDParam)((currentParam + 1) % 3);
      beepTimes(currentParam + 1, 400);
      pidLastSwitch = millis();
    } else {
      switch (currentMode) {
        case ANGLE:
          if (currentParam == PARAM_KP) adjustValue(angleKp, kpMin, kpMax, kpStep, EEPROM_ANGLE_KP_ADDR);
          else if (currentParam == PARAM_KI) adjustValue(angleKi, kiMin, kiMax, kiStep, EEPROM_ANGLE_KI_ADDR);
          else adjustValue(angleKd, kdMin, kdMax, kdStep, EEPROM_ANGLE_KD_ADDR);
          break;
        case RATE:
          adjustValue(rateKd, kdMin, kdMax, kdStep, EEPROM_RATE_KD_ADDR);
          break;
        case ADAPTIVE:
          if (currentParam == PARAM_KP) adjustValue(adaptKp, kpMin, kpMax, kpStep, EEPROM_ADAPT_KP_ADDR);
          else if (currentParam == PARAM_KI) adjustValue(adaptKi, kiMin, kiMax, kiStep, EEPROM_ADAPT_KI_ADDR);
          else adjustValue(adaptKd, kdMin, kdMax, kdStep, EEPROM_ADAPT_KD_ADDR);
          break;
      }
    }
  }
  lastState = currentState;
}

void adjustValue(float &value, float minVal, float maxVal, float step, int addr) {
  value += step;
  if (value > maxVal) {
    value = minVal;
    tone(BUZZER_PIN, 500, 1000);
    delay(1000);
  } else {
    tone(BUZZER_PIN, 2000, 100);
    delay(100);
  }
  EEPROM.put(addr, value);
  digitalWrite(LED_PIN, HIGH); delay(100); digitalWrite(LED_PIN, LOW);
}

// ----------------- Support --------------------

void beepTimes(int count, int duration) {
  for (int i = 0; i < count; i++) {
    tone(BUZZER_PIN, 1500, duration);
    delay(duration + 100);
  }
}

void motorWrite() {
  if (millis() - timestamp >= 20) {
    timestamp = millis();
    PORTC |= B00000001;
    long esc_timer = micros();
    while (PORTC & B00000001) {
      if (micros() >= (steer + esc_timer)) {
        PORTC &= ~B00000001;
      }
    }
  }
}

void signalWeightLevel() {
  if (currentWeightIndex == NUM_WEIGHTS - 1) {
    digitalWrite(LED_PIN, HIGH); delay(650); digitalWrite(LED_PIN, LOW);
  } else {
    for (int j = 0; j <= currentWeightIndex; j++) {
      digitalWrite(LED_PIN, HIGH); delay(200);
      digitalWrite(LED_PIN, LOW); delay(200);
    }
  }
}

void indicateMode() {
  switch (currentMode) {
    case RATE: beepTimes(2, 100); break;
    case ANGLE: beepTimes(1, 300); break;
    case ADAPTIVE: beepTimes(3, 200); break;
  }
}

void calibrateGyro() {
  digitalWrite(LED_PIN, HIGH);
  offsetGz = 0;
  for (int i = 0; i < SAMPLE; i++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    offsetGz += gz;
    delay(5);
  }
  offsetGz /= SAMPLE;
  digitalWrite(LED_PIN, LOW);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW); delay(100);
  }
}

ISR(PCINT2_vect) {
  unsigned long now = micros();
  if (last_channel == 0 && (PIND & (1 << PIND4))) {
    last_channel = 1;
    timer[0] = now;
  } else if (last_channel == 1 && !(PIND & (1 << PIND4))) {
    last_channel = 0;
    input = now - timer[0];
  }
}
