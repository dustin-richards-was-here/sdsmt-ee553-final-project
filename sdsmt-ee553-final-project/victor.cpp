#include <Servo.h>
#include <Arduino.h>
#include "victor.h"

Victor::Victor(int _pin)
{
  pin = _pin;
}

void Victor::init()
{
  servo.attach(pin);
}

void Victor::setPower(float power)
{
  power *= (Victor::SERVO_MAX_US - Victor::SERVO_MIN_US) / 100 / 2;
  power += Victor::SERVO_MID_US;
  servo.writeMicroseconds(int(power));
}