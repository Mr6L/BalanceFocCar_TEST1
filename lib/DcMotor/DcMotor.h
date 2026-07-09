#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <Arduino.h>

class DcMotor
{
  public:
    DcMotor();

    // pwmPin: LEDC PWM output to TB6612 PWMA/PWMB
    // in1Pin, in2Pin: direction inputs
    // ledcChannel: LEDC channel (0-15 on ESP32-S3)
    // sign: +1 or -1 to flip mechanical forward direction
    void init(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin,
              uint8_t ledcChannel, float sign = 1.0f);

    // speed: -1.0 .. 1.0
    void setSpeed(float speed);
    void brake();
    void coast();

    float getLastCommand() const { return lastCommand_; }

  private:
    uint8_t pwmPin_ = 0;
    uint8_t in1Pin_ = 0;
    uint8_t in2Pin_ = 0;
    uint8_t ledcChannel_ = 0;
    float sign_ = 1.0f;
    float lastCommand_ = 0.0f;
    bool initialized_ = false;

    void writePwm(uint8_t duty);
};

#endif
