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
Victor travelMotor(travelMotorPin, 1);
Victor liftMotor(liftMotorPin, 0);
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
// diameter of travel pulley in mm
constexpr float TRAVEL_PULLEY_DIAMETER = 22.3;
// distance carriage travels for each revolution of the pulley, mm
constexpr float TRAVEL_MM_PER_REV = PI * TRAVEL_PULLEY_DIAMETER;
// convert an ADC reading to an angle in radians
constexpr float ADC_TO_RADIANS(int adc) {return 0.00507 * (adc-452);}

// filtering for joystick inputs
constexpr int joystickXFilterCutoffFreq = 1;
constexpr int joystickYFilterCutoffFreq = 10;
FilterOnePole joystickXFilter(LOWPASS, joystickXFilterCutoffFreq);
FilterOnePole joystickYFilter(LOWPASS, joystickYFilterCutoffFreq);

// travel controller setup
double KpTravelVelocity = 0.5;
double KiTravelVelocity = 1;
double KdTravelVelocity = 0;
double KpTravelPosition = 3;
double KiTravelPosition = 1;
double KdTravelPosition = 0;
PID_v2 travelVelocityPID(KpTravelVelocity, KiTravelVelocity, KdTravelVelocity, PID::Direct);
PID_v2 travelPositionPID(KpTravelPosition, KiTravelPosition, KdTravelPosition, PID::Direct);

// swing controller setup
double KpSwing = 50;
double KiSwing = 0;
double KdSwing = 2;
PID_v2 swingPID(KpSwing, KiSwing, KdSwing, PID::Direct);

// balances the contribution of the travel position and swing control loops
// higher -> more travel position
//float alpha = 0.08;
float alpha = 0.08;
  
// zero the travel axis
void zeroTravel()
{
  travelVelocityPID.Setpoint(-100);

  while (digitalRead(endstopPin)) {
    float travelVelocity = -travelEncoder.getAngularSpeed() / 360 * TRAVEL_MM_PER_REV;
    const double travelPower = constrain(travelVelocityPID.Run(travelVelocity) / PID_OUTPUT_LIMIT, -0.25, 0.25);
    travelMotor.setPower(travelPower);
  }

  travelMotor.setPower(0);
  travelVelocityPID.Setpoint(0);

  travelEncoder.resetCumulativePosition();
}

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
  travelEncoder.resetCumulativePosition();

  // PID setup
  travelVelocityPID.SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  travelVelocityPID.SetSampleTime(10);
  travelVelocityPID.Start(0, 0, 0);
  travelPositionPID.SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  travelPositionPID.SetSampleTime(10);
  travelPositionPID.Start(0, 0, 0);
  swingPID.SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  swingPID.SetSampleTime(10);
  swingPID.Start(0, 0, 0);
  
  pinMode(endstopPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  zeroTravel();

  Serial.println("BEGIN");
  Serial.println("\"time (ms)\", \"filtered velocity (deg/s)\", \"travel power (%)\", \"load angle ADC\"");
}

// logging interval parameters
constexpr int TRAVEL_VELOCITY_REPORT_PERIOD = 50; // ms
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
  // ===== LOAD CELL =====
  // float weight = loadCell.get_units(10);
  // Serial.println(weight);

  // ===== TRAVEL ENCODER =====
  // input mm/s into the filter
  float travelVelocity = -travelEncoder.getAngularSpeed() / 360 * TRAVEL_MM_PER_REV;
  float travelPosition = -(float)travelEncoder.getCumulativePosition() / 4096 * TRAVEL_MM_PER_REV;

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
  joystickXFilter.input(stickToPower(joystickXVal, 0.05));
  joystickYFilter.input(-stickToPower(joystickYVal, 0.05));
  travelPositionPID.Setpoint(-joystickXFilter.Y * 450 + 600);
  //travelPositionPID.Setpoint(-joystickXFilter.Y * 500);
  const double travelPowerStick = travelPositionPID.Run(travelPosition) / PID_OUTPUT_LIMIT;
  liftMotor.setPower(joystickYFilter.Y);

  // ===== SWING ANGLE CONTROLLER =====
  const float swingAngle = ADC_TO_RADIANS(analogRead(loadAnglePotPin));
  const double travelPowerSwing = swingPID.Run(swingAngle) / PID_OUTPUT_LIMIT;

  // ===== OUTPUT TO TRAVEL MOTOR =====
  float travelPower = ((travelPowerStick * alpha) + (travelPowerSwing * (1-alpha))) * 2;
  travelMotor.setPower(constrain(travelPower, -0.25, 0.25));

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
      //Serial.print(now);
      //Serial.print(",");
      //Serial.println(swingAngle);
      //Serial.print(travelPowerStick);
      //Serial.print(",");
      //Serial.print(travelPowerSwing);
      //Serial.print(",");
      //Serial.print(travelPosition);
      //Serial.println();
    }

    lastTravelVelocityReportTime = millis();
  }

}