#include "HX711.h"

#define LOADCELL1_DOUT_PIN 3
#define LOADCELL1_SCK_PIN 2
#define LOADCELL2_DOUT_PIN 5
#define LOADCELL2_SCK_PIN 4

#define MOTOR_PWM_PIN 9
#define MOTOR_IN1_PIN 7
#define MOTOR_IN2_PIN 8

#define LED_GREEN_PIN 10
#define LED_YELLOW_PIN 11
#define LED_RED_PIN 12

#define EMERGENCY_BUTTON_PIN 6
#define POWER_SWITCH_PIN 13

#define PRESSURE_TARGET 18.0
#define PRESSURE_SAFE_MAX 20.0
#define PRESSURE_CAUTION_MAX 28.0
#define PRESSURE_WARNING_MAX 32.0
#define PRESSURE_CRITICAL 32.0

#define RISK_SCORE_LIMIT 600.0
#define HOTSPOT_THRESHOLD 8.0

#define KP 0.8
#define KI 0.1
#define CYCLE_TIME_MS 50
#define MAX_MOTOR_SPEED 255
#define MAX_STEPS_PER_CYCLE 10

#define LOADCELL1_CALIBRATION -7050.0
#define LOADCELL2_CALIBRATION -7050.0

#define GRAMS_TO_MMHG 0.00386

HX711 loadcell1;
HX711 loadcell2;

float pressure1 = 0.0;
float pressure2 = 0.0;
float pressureAvg = 0.0;
float pressureMax = 0.0;

float integralError = 0.0;
float riskScore = 0.0;

unsigned long lastCycleTime = 0;

bool systemEnabled = false;
bool emergencyTriggered = false;

enum RiskZone { SAFE, CAUTION, WARNING, CRITICAL };
RiskZone currentZone = SAFE;

void setup() {
  Serial.begin(9600);
  loadcell1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
  loadcell2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);
  loadcell1.set_scale(LOADCELL1_CALIBRATION);
  loadcell2.set_scale(LOADCELL2_CALIBRATION);
  loadcell1.tare();
  loadcell2.tare();
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  stopMotor();
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(EMERGENCY_BUTTON_PIN, INPUT_PULLUP);
  pinMode(POWER_SWITCH_PIN, INPUT_PULLUP);
  digitalWrite(LED_GREEN_PIN, HIGH);
  delay(200);
  digitalWrite(LED_YELLOW_PIN, HIGH);
  delay(200);
  digitalWrite(LED_RED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  Serial.println("System Ready. Flip power switch to activate.");
}

void loop() {
  unsigned long currentTime = millis();
  if (digitalRead(EMERGENCY_BUTTON_PIN) == LOW) {
    emergencyRelease();
    return;
  }
  systemEnabled = (digitalRead(POWER_SWITCH_PIN) == HIGH);
  if (!systemEnabled) {
    stopMotor();
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_YELLOW_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    delay(100);
    return;
  }
  if (currentTime - lastCycleTime < CYCLE_TIME_MS) return;
  lastCycleTime = currentTime;
  readPressureSensors();
  calculatePressureStats();
  updateRiskScore();
  classifyRiskZone();
  float motorCommand = calculateMotorCommand();
  executeMotorCommand(motorCommand);
  updateStatusLEDs();
  printStatus();
}

void readPressureSensors() {
  if (loadcell1.is_ready() && loadcell2.is_ready()) {
    float force1 = loadcell1.get_units(3);
    float force2 = loadcell2.get_units(3);
    pressure1 = max(0.0, force1 * GRAMS_TO_MMHG);
    pressure2 = max(0.0, force2 * GRAMS_TO_MMHG);
  } else {
    Serial.println("ERROR: Load cell not ready");
  }
}

void calculatePressureStats() {
  pressureAvg = (pressure1 + pressure2) / 2.0;
  pressureMax = max(pressure1, pressure2);
}

void updateRiskScore() {
  float timeDelta = CYCLE_TIME_MS / 1000.0 / 60.0;
  if (pressure1 > PRESSURE_SAFE_MAX) riskScore += (pressure1 - PRESSURE_SAFE_MAX) * timeDelta;
  if (pressure2 > PRESSURE_SAFE_MAX) riskScore += (pressure2 - PRESSURE_SAFE_MAX) * timeDelta;
  if (pressureMax < PRESSURE_SAFE_MAX) riskScore *= 0.998;
  riskScore = max(0.0, riskScore);
}

void classifyRiskZone() {
  if (pressureMax > PRESSURE_CRITICAL || riskScore > RISK_SCORE_LIMIT) currentZone = CRITICAL;
  else if (pressureMax > PRESSURE_WARNING_MAX) currentZone = WARNING;
  else if (pressureMax > PRESSURE_CAUTION_MAX) currentZone = CAUTION;
  else currentZone = SAFE;
}

float calculateMotorCommand() {
  float error = PRESSURE_TARGET - pressureAvg;
  integralError += error * (CYCLE_TIME_MS / 1000.0);
  integralError = constrain(integralError, -50.0, 50.0);
  float baseCommand = KP * error + KI * integralError;
  float pressureDifference = abs(pressure1 - pressure2);
  if (pressureDifference > HOTSPOT_THRESHOLD) baseCommand = min(baseCommand, -0.5 * (pressureMax - PRESSURE_WARNING_MAX));
  float motorCommand = 0.0;
  switch (currentZone) {
    case CRITICAL: motorCommand = max(baseCommand, -20.0); break;
    case WARNING: motorCommand = min(baseCommand, 0.0); break;
    case CAUTION: motorCommand = constrain(baseCommand, -5.0, 0.0); break;
    case SAFE: motorCommand = constrain(baseCommand, -10.0, 10.0); break;
  }
  motorCommand = constrain(motorCommand, -MAX_STEPS_PER_CYCLE, MAX_STEPS_PER_CYCLE);
  return motorCommand;
}

void executeMotorCommand(float command) {
  if (abs(command) < 0.5) { stopMotor(); return; }
  int motorSpeed = map(abs(command), 0, MAX_STEPS_PER_CYCLE, 80, MAX_MOTOR_SPEED);
  motorSpeed = constrain(motorSpeed, 0, MAX_MOTOR_SPEED);
  if (command < 0) { digitalWrite(MOTOR_IN1_PIN, LOW); digitalWrite(MOTOR_IN2_PIN, HIGH); }
  else { digitalWrite(MOTOR_IN1_PIN, HIGH); digitalWrite(MOTOR_IN2_PIN, LOW); }
  analogWrite(MOTOR_PWM_PIN, motorSpeed);
  delay(abs(command) * 10);
  stopMotor();
}

void stopMotor() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  analogWrite(MOTOR_PWM_PIN, 0);
}

void emergencyRelease() {
  Serial.println("!!! EMERGENCY RELEASE ACTIVATED !!!");
  emergencyTriggered = true;
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, HIGH);
  analogWrite(MOTOR_PWM_PIN, MAX_MOTOR_SPEED);
  for (int i = 0; i < 30; i++) { digitalWrite(LED_RED_PIN, HIGH); delay(50); digitalWrite(LED_RED_PIN, LOW); delay(50); }
  stopMotor();
  riskScore = 0.0;
  integralError = 0.0;
  Serial.println("Emergency release complete. Power cycle to resume.");
  while(true) { digitalWrite(LED_RED_PIN, HIGH); delay(500); digitalWrite(LED_RED_PIN, LOW); delay(500); }
}

void updateStatusLEDs() {
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  switch (currentZone) {
    case SAFE: digitalWrite(LED_GREEN_PIN, HIGH); break;
    case CAUTION:
    case WARNING: digitalWrite(LED_YELLOW_PIN, HIGH); break;
    case CRITICAL: digitalWrite(LED_RED_PIN, HIGH); break;
  }
}

void printStatus() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print("Zone: ");
    switch(currentZone) { case SAFE: Serial.print("SAFE    "); break; case CAUTION: Serial.print("CAUTION "); break; case WARNING: Serial.print("WARNING "); break; case CRITICAL: Serial.print("CRITICAL"); break; }
    Serial.print(" | P1: "); Serial.print(pressure1, 1);
    Serial.print(" | P2: "); Serial.print(pressure2, 1);
    Serial.print(" | Avg: "); Serial.print(pressureAvg, 1);
    Serial.print(" | Max: "); Serial.print(pressureMax, 1);
    Serial.print(" | Risk: "); Serial.println(riskScore, 1);
  }
}
