#include "DcMotor.h"

static const uint32_t PWM_FREQ_HZ = 20000;
static const uint8_t  PWM_RES_BITS = 8;
static const uint16_t PWM_MAX_DUTY = 255;

DcMotor::DcMotor() = default;

void DcMotor::init(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin,
                   uint8_t ledcChannel, float sign)
{
    pwmPin_ = pwmPin;
    in1Pin_ = in1Pin;
    in2Pin_ = in2Pin;
    ledcChannel_ = ledcChannel;
    sign_ = sign >= 0.0f ? 1.0f : -1.0f;

    pinMode(pwmPin_, OUTPUT);
    pinMode(in1Pin_, OUTPUT);
    pinMode(in2Pin_, OUTPUT);

    ledcSetup(ledcChannel_, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(pwmPin_, ledcChannel_);

    brake();
    initialized_ = true;
}

void DcMotor::writePwm(uint8_t duty)
{
    ledcWrite(ledcChannel_, duty);
}

void DcMotor::setSpeed(float speed)
{
    if (!initialized_)
    {
        return;
    }

    speed *= sign_;
    speed = constrain(speed, -1.0f, 1.0f);
    lastCommand_ = speed;

    const float absSpeed = fabsf(speed);
    const uint8_t duty = (uint8_t)(absSpeed * (float)PWM_MAX_DUTY);

    if (duty == 0)
    {
        brake();
        return;
    }

    if (speed > 0.0f)
    {
        digitalWrite(in1Pin_, HIGH);
        digitalWrite(in2Pin_, LOW);
    }
    else
    {
        digitalWrite(in1Pin_, LOW);
        digitalWrite(in2Pin_, HIGH);
    }

    writePwm(duty);
}

void DcMotor::brake()
{
    digitalWrite(in1Pin_, HIGH);
    digitalWrite(in2Pin_, HIGH);
    writePwm(0);
    lastCommand_ = 0.0f;
}

void DcMotor::coast()
{
    digitalWrite(in1Pin_, LOW);
    digitalWrite(in2Pin_, LOW);
    writePwm(0);
    lastCommand_ = 0.0f;
}
