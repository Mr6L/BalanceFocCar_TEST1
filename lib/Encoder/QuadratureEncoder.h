#ifndef QUADRATURE_ENCODER_H
#define QUADRATURE_ENCODER_H

#include <Arduino.h>
#include <ESP32Encoder.h>

class QuadratureEncoder
{
  public:
    QuadratureEncoder();

    bool init(int8_t pinA, int8_t pinB, float countsPerRevolution, float sign = 1.0f);

    int64_t getCount();
    float getAngle();            // output shaft radians
    float getVelocity();         // output shaft rad/s
    float getRpm() const;        // output shaft RPM

    void reset();

  private:
    ESP32Encoder encoder_;
    float countsPerRev_ = 1.0f;
    float sign_ = 1.0f;
    bool initialized_ = false;

    int64_t lastCount_ = 0;
    uint32_t lastUpdateUs_ = 0;
    float lastVelocity_ = 0.0f;
};

#endif
