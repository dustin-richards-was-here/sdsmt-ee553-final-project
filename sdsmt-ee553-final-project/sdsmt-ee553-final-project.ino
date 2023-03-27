#include "victor.h"

constexpr int joystickXPin = A0;

Victor victor(53);

void setup() {
  // put your setup code here, to run once:
  victor.init();

  pinMode(joystickXPin, INPUT);

  Serial.begin(115200);
}

int value = 0;
int increment = 1;
int minValue = -100;
int maxValue = 100;

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
  int joystickXValue = analogRead(joystickXPin);
  float value = (float(joystickXValue) / 1023 * 200) - 100;
  Serial.println(value);

  victor.setPower(value);
}