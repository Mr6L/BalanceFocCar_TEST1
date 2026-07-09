#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID
{
  public:
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f,
        float outputLimit = 1.0e6f, float integralLimit = 1.0e6f)
        : kp_(kp), ki_(ki), kd_(kd),
          outputLimit_(outputLimit), integralLimit_(integralLimit),
          integral_(0.0f), prevError_(0.0f), first_(true)
    {
    }

    void setGains(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void getGains(float &kp, float &ki, float &kd) const
    {
        kp = kp_;
        ki = ki_;
        kd = kd_;
    }

    void setOutputLimit(float limit)
    {
        outputLimit_ = limit;
    }

    void setIntegralLimit(float limit)
    {
        integralLimit_ = limit;
    }

    float update(float error, float dt)
    {
        if (dt <= 0.0f)
        {
            return 0.0f;
        }

        integral_ += error * dt;
        if (integralLimit_ > 0.0f)
        {
            integral_ = constrain(integral_, -integralLimit_, integralLimit_);
        }

        float derivative = 0.0f;
        if (!first_)
        {
            derivative = (error - prevError_) / dt;
        }
        prevError_ = error;
        first_ = false;

        float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        if (outputLimit_ > 0.0f)
        {
            output = constrain(output, -outputLimit_, outputLimit_);
        }
        return output;
    }

    void reset()
    {
        integral_ = 0.0f;
        prevError_ = 0.0f;
        first_ = true;
    }

  private:
    float kp_, ki_, kd_;
    float outputLimit_, integralLimit_;
    float integral_;
    float prevError_;
    bool first_;
};

#endif
