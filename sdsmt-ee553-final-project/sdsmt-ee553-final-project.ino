#include "victor.h"
#include "HX711.h"

// pin definitions
constexpr int joystickXPin = A0;
constexpr int endstopPin = 14;
constexpr int loadCellDoutPin = 2;
constexpr int loadCellSckPin = 3;

// device objects
Victor victor(53);
HX711 loadCell;

// load cell calibration
// number acquired from get_units(50) with a 7lb load + a belt: 205674.00
// the actual mass of this load: 3182.5 grams
constexpr float loadcellScale = 205764.00 / 3.1825; // scale output to kg

void setup() {
  Serial.begin(115200);

  victor.init();

  // load cell setup
  loadCell.begin(loadCellDoutPin, loadCellSckPin);
  loadCell.set_scale(loadcellScale);
  float preTareWeight = loadCell.get_units(10);
  Serial.print("Load cell tared @ ");
  Serial.print(preTareWeight);
  Serial.println(" kg");
  loadCell.tare();

  pinMode(joystickXPin, INPUT);
  pinMode(endstopPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

// motor sweep values
int value = 0;
int increment = 1;
const int minValue = -100;
const int maxValue = 100;

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
  float weight = loadCell.get_units(10);
  Serial.println(weight);
}