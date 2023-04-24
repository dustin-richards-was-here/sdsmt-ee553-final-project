#include "victor.h"
#include "HX711.h"
#include "AS5600.h"
#include "Filters.h"
#include "PID_v2.h"
#include "Wire.h"

//#define DEBUG

// pin definitions
constexpr int joystickXPin = A2;
constexpr int joystickYPin = A1;
constexpr int endstopPin = A0;
constexpr int loadCellDoutPin = 2;
constexpr int loadCellSckPin = 3;
constexpr int encoderPowerPin = 10;
constexpr int travelMotorPin = 11;
constexpr int liftMotorPin = 12;
constexpr int loadAnglePotPin = A3;

// device objects
Victor travelMotor(travelMotorPin);
Victor liftMotor(liftMotorPin);
HX711 loadCell;
TwoWire *travelEncoderI2C = &Wire;
TwoWire *liftEncoderI2C = &Wire1;
AS5600 travelEncoder(travelEncoderI2C);
AS5600 liftEncoder(liftEncoderI2C);

// load cell calibration
// number acquired from get_units(50) with a 7lb load + a belt: 205674.00
// the actual mass of this load: 3182.5 grams
constexpr float loadcellScale = 205764.00 / 3.1825; // scale output to kg

// useful constants
// needed because the AS5600 library outputs in deg/sec
constexpr float DPS_TO_HZ = 1 / 360;
// needed because the AS5600 library outputs in deg/sec
constexpr float HZ_TO_DPS = 360;
// approx max speed with no load @ 12V
constexpr float TRAVEL_VELOCITY_MAX_HZ = 100;
constexpr float TRAVEL_VELOCITY_MAX_DPS = TRAVEL_VELOCITY_MAX_HZ * HZ_TO_DPS;
// because the PID class outputs an integer typecast to a float??
constexpr int PID_OUTPUT_LIMIT = 1023;

// filtering for encoder velocity esimation
constexpr int velocityFilterCutoffFreq = 100; // Hz, I think
FilterOnePole travelVelocityFilter(LOWPASS, velocityFilterCutoffFreq);

// filtering for joystick inputs
constexpr int joystickFilterCutoffFreq = 10;
FilterOnePole joystickXFilter(LOWPASS, joystickFilterCutoffFreq);
FilterOnePole joystickYFilter(LOWPASS, joystickFilterCutoffFreq);

// PID setup
//double KpTravelVelocity = 0.00005;
double KpTravelVelocity = 0.03;
double KiTravelVelocity = 0.03;
double KdTravelVelocity = 0;
PID_v2 travelVelocityPID(KpTravelVelocity, KiTravelVelocity, KdTravelVelocity, PID::Direct);

void setup() {
  travelMotor.init();
  liftMotor.init();

  Serial.begin(115200);

  // load cell setup
  //loadCell.begin(loadCellDoutPin, loadCellSckPin);
  //loadCell.set_scale(loadcellScale);
  //float preTareWeight = loadCell.get_units(10);
  //Serial.print("Load cell tared @ ");
  //Serial.print(preTareWeight);
  //Serial.println(" kg");
  //loadCell.tare();

  // encoder setup
  pinMode(encoderPowerPin, OUTPUT);
  digitalWrite(encoderPowerPin, 1);
  delay(250);
  travelEncoder.begin();
  liftEncoder.begin();
  while (!travelEncoder.isConnected() || !liftEncoder.isConnected()) {
    #ifdef DEBUG
    Serial.println("failed to connect to AS5600, power cycling");
    #endif // DEBUG
    digitalWrite(encoderPowerPin, 0);
    delay(250);
    digitalWrite(encoderPowerPin, 1);
    delay(250);
    travelEncoder.begin();
    liftEncoder.begin();
  }

  // PID setup
  travelVelocityPID.SetOutputLimits(0, PID_OUTPUT_LIMIT);
  travelVelocityPID.Start(0, 0, 30 * HZ_TO_DPS);
  
  pinMode(endstopPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("BEGIN");
  Serial.println("\"time (ms)\", \"filtered velocity (deg/s)\", \"travel power (%)\", \"load angle ADC\"");
}

// motor sweep values
int value = 0;
int increment = 1;
const int minValue = -100;
const int maxValue = 100;

// travel velocity reporting
constexpr int TRAVEL_VELOCITY_REPORT_PERIOD = 2; // ms
uint32_t lastTravelVelocityReportTime = -99999;

inline float stickToPower(int joystickVal, float deadzone)
{
  float power = (float)joystickVal / (1023/2) - 1;
  if (abs(power) < deadzone) {
    power = 0;
  }
  return power;
}

void loop() {
  // ===== MOTOR SWEEP =====
  // if (value >= maxValue || value <= minValue)
  // {
  //   increment *= -1;
  //   delay(5000);
  // }
  //
  // value += increment;
  //
  // delay(100);

  // ===== MOTOR W/ JOYSTICK =====
  // int joystickXValue = analogRead(joystickXPin);
  // float value = (float(joystickXValue) / 1023 * 200) - 100;

  // victor.setPower(value);

  // // ===== INDUCTIVE ENDSTOP =====
  // int endstopValue = digitalRead(endstopPin);
  // digitalWrite(LED_BUILTIN, !endstopValue);

  // ===== LOAD CELL =====
  // float weight = loadCell.get_units(10);
  // Serial.println(weight);

  // ===== TRAVEL ENCODER =====
  travelVelocityFilter.input(-travelEncoder.getAngularSpeed());

  // ===== TRAVEL VELOCITY PID =====
  //const double travelPower = travelVelocityPID.Run(travelVelocityFilter.Y) / PID_OUTPUT_LIMIT;
  //victor.setPower(travelPower);

  // ===== STEP INPUT CONTROL =====
  //int joystickVal = analogRead(joystickXPin);
  //float travelPower = 0;
  //if (joystickVal > 1000) {
  //  travelPower = 0.25;
  //} else if (joystickVal < 10) {
  //  travelPower = -0.25;
  //}
  //travelMotor.setPower(travelPower);
  
  // ===== GENERAL JOYSTICK CONTROL =====
  int joystickXVal = analogRead(joystickXPin);
  int joystickYVal = analogRead(joystickYPin);
  joystickXFilter.input(stickToPower(joystickXVal, 0) / 4);
  joystickYFilter.input(-stickToPower(joystickYVal, 0.01));
  travelMotor.setPower(joystickXFilter.Y);
  liftMotor.setPower(joystickYFilter.Y);

  uint32_t now = millis();
  if (now - lastTravelVelocityReportTime > TRAVEL_VELOCITY_REPORT_PERIOD) {
    if (!travelEncoder.isConnected()) {
      #ifdef DEBUG
      Serial.println("lost connection to AS5600");
      #endif // DEBUG
      Serial.print(9999);
      Serial.print(", ");
      Serial.print(9999);
      Serial.println();
    } else {
      Serial.print(now);
      Serial.print(",");
      Serial.print(joystickXVal);
      Serial.print(",");
      Serial.print(joystickYVal);
      Serial.println();
    }

    lastTravelVelocityReportTime = millis();
  }

}