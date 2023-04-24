#ifndef VICTOR__H
#define VICTOR__H

#include <Servo.h>

class Victor
{
public:
  Victor(int _pin, bool reverse);
  void init();
  void setPower(float power);

  static constexpr int SERVO_MIN_US = 1000;
  static constexpr int SERVO_MID_US = 1500;
  static constexpr int SERVO_MAX_US = 2000;

private:
  int pin;
  int reverse;
  Servo servo;
};

#endif //VICTOR__H