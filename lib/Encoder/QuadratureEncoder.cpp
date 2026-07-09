#include "QuadratureEncoder.h"

#include <math.h>

QuadratureEncoder::QuadratureEncoder() = default;

bool QuadratureEncoder::init(int8_t pinA, int8_t pinB, float countsPerRevolution, float sign)
{
    if (countsPerRevolution <= 0.0f)
    {
        return false;
    }

    countsPerRev_ = countsPerRevolution;
    sign_ = sign >= 0.0f ? 1.0f : -1.0f;

    // Enable internal pull-ups for open-collector Hall encoders.
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);

    encoder_.attachFullQuad(pinA, pinB);
    encoder_.setCount(0);

    lastCount_ = 0;
    lastUpdateUs_ = micros();
    lastVelocity_ = 0.0f;
    initialized_ = true;

    return true;
}

int64_t QuadratureEncoder::getCount()
{
    if (!initialized_)
    {
        return 0;
    }
    return encoder_.getCount();
}

float QuadratureEncoder::getAngle()
{
    if (!initialized_)
    {
        return 0.0f;
    }
    const float revolutions = (float)getCount() / countsPerRev_;
    return revolutions * 2.0f * PI * sign_;
}

float QuadratureEncoder::getVelocity()
{
    if (!initialized_)
    {
        return 0.0f;
    }

    const int64_t count = encoder_.getCount();
    const uint32_t nowUs = micros();

    // Compute dt in seconds, handling micros() overflow.
    const uint32_t deltaUs = nowUs - lastUpdateUs_;
    if (deltaUs < 500)
    {
        return lastVelocity_;
    }

    const int64_t deltaCount = count - lastCount_;
    const float dt = (float)deltaUs / 1e6f;

    const float revolutions = (float)deltaCount / countsPerRev_;
    const float radPerSec = (revolutions * 2.0f * PI / dt) * sign_;

    lastCount_ = count;
    lastUpdateUs_ = nowUs;
    lastVelocity_ = radPerSec;

    return radPerSec;
}

float QuadratureEncoder::getRpm() const
{
    const float radPerSec = lastVelocity_;
    return radPerSec * 60.0f / (2.0f * PI);
}

void QuadratureEncoder::reset()
{
    if (!initialized_)
    {
        return;
    }
    encoder_.setCount(0);
    lastCount_ = 0;
    lastUpdateUs_ = micros();
    lastVelocity_ = 0.0f;
}
