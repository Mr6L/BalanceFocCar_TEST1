// Copy this file to src/main.cpp for a standalone quadrature encoder test.

#include <Arduino.h>
#include "QuadratureEncoder.h"
#include "UserConfig.h"

QuadratureEncoder encoderA;
QuadratureEncoder encoderB;

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("Encoder test start");

    const float countsPerRev = (float)MOTOR_ENCODER_PPR * 4.0f * MOTOR_GEAR_RATIO;
    encoderA.init(MOTOR_A_ENC_A, MOTOR_A_ENC_B, countsPerRev, MOTOR_A_BALANCE_SIGN);
    encoderB.init(MOTOR_B_ENC_A, MOTOR_B_ENC_B, countsPerRev, MOTOR_B_BALANCE_SIGN);
}

void loop()
{
    Serial.printf("A: count=%lld angle=%.2f vel=%.3f | B: count=%lld angle=%.2f vel=%.3f\n",
                  encoderA.getCount(), encoderA.getAngle(), encoderA.getVelocity(),
                  encoderB.getCount(), encoderB.getAngle(), encoderB.getVelocity());
    delay(200);
}
