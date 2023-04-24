#include <Servo.h>
#include <Arduino.h>
#include "victor.h"

Victor::Victor(int _pin, bool _reverse)
{
  pin = _pin;

  if (_reverse) {
    reverse = -1;
  } else {
    reverse = 1;
  }
}

void Victor::init()
{
  servo.attach(pin);
}

void Victor::setPower(float power)
{
  power *= (Victor::SERVO_MAX_US - Victor::SERVO_MIN_US) / 2 * reverse;
  power += Victor::SERVO_MID_US;
  servo.writeMicroseconds(int(power));
}