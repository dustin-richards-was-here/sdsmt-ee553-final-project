#include "victor.h"
#include "HX711.h"
#include "AS5600.h"
#include "Filters.h"
#include "PID_v2.h"
#include "Wire.h"

// pin definitions
constexpr int joystickXPin = A1;
constexpr int endstopPin = 14;
constexpr int loadCellDoutPin = 2;
constexpr int loadCellSckPin = 3;
constexpr int travelEncoderPowerPin = A0;

// device objects
Victor victor(53);
HX711 loadCell;
TwoWire *travelEncoderI2C = &Wire1;
AS5600 travelEncoder(travelEncoderI2C);

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
constexpr int velocityFilterCutoffFreq = 5; // Hz, I think
FilterOnePole travelVelocityFilter(LOWPASS, velocityFilterCutoffFreq);

// PID setup
//double KpTravelVelocity = 0.00005;
double KpTravelVelocity = 0.03;
double KiTravelVelocity = 0.03;
double KdTravelVelocity = 0;
PID_v2 travelVelocityPID(KpTravelVelocity, KiTravelVelocity, KdTravelVelocity, PID::Direct);

void setup() {
  Serial.begin(115200);

  victor.init();

  // load cell setup
  // loadCell.begin(loadCellDoutPin, loadCellSckPin);
  // loadCell.set_scale(loadcellScale);
  // float preTareWeight = loadCell.get_units(10);
  // Serial.print("Load cell tared @ ");
  // Serial.print(preTareWeight);
  // Serial.println(" kg");
  // loadCell.tare();

  // encoder setup
  pinMode(travelEncoderPowerPin, OUTPUT);
  digitalWrite(travelEncoderPowerPin, 1);
  travelEncoder.begin();
  while (!travelEncoder.isConnected()) {
    Serial.println("failed to connect to AS5600, power cycling");
    digitalWrite(travelEncoderPowerPin, 0);
    delay(250);
    digitalWrite(travelEncoderPowerPin, 1);
    delay(250);
    travelEncoder.begin();
  }

  // PID setup
  travelVelocityPID.SetOutputLimits(0, PID_OUTPUT_LIMIT);
  travelVelocityPID.Start(0, 0, 30 * HZ_TO_DPS);
  
  pinMode(joystickXPin, INPUT);
  pinMode(endstopPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

// motor sweep values
int value = 0;
int increment = 1;
const int minValue = -100;
const int maxValue = 100;

// travel velocity reporting
constexpr int TRAVEL_VELOCITY_REPORT_PERIOD = 100; // ms
uint32_t lastTravelVelocityReportTime = 0;

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
  travelVelocityFilter.input(travelEncoder.getAngularSpeed());

  // ===== TRAVEL VELOCITY PID =====
  const double travelPower = travelVelocityPID.Run(travelVelocityFilter.Y) / PID_OUTPUT_LIMIT;
  victor.setPower(travelPower);

  if (millis() - lastTravelVelocityReportTime > TRAVEL_VELOCITY_REPORT_PERIOD) {
    if (!travelEncoder.isConnected()) {
      Serial.println("lost connection to AS5600");
    } else {
      Serial.print(travelVelocityFilter.Y);
      Serial.print("\t\t");
      Serial.print(travelVelocityPID.GetSetpoint());
      Serial.print("\t\t");
      Serial.print(travelPower);
      Serial.println();
    }

    lastTravelVelocityReportTime = millis();
  }

}