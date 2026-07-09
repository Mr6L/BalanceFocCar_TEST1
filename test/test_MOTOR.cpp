// Copy this file to src/main.cpp for a standalone DC motor test.

#include <Arduino.h>
#include "DcMotor.h"
#include "UserConfig.h"

DcMotor motorA;
DcMotor motorB;

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("DC motor test start");

    motorA.init(MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, 0, MOTOR_A_BALANCE_SIGN);
    motorB.init(MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, 1, MOTOR_B_BALANCE_SIGN);
}

static void rampMotor(DcMotor &motor, const char *name)
{
    Serial.printf("%s forward\n", name);
    for (float s = 0.0f; s <= 0.5f; s += 0.05f)
    {
        motor.setSpeed(s);
        delay(200);
    }
    delay(500);

    Serial.printf("%s brake\n", name);
    motor.brake();
    delay(500);

    Serial.printf("%s reverse\n", name);
    for (float s = 0.0f; s <= 0.5f; s += 0.05f)
    {
        motor.setSpeed(-s);
        delay(200);
    }
    delay(500);

    Serial.printf("%s coast\n", name);
    motor.coast();
    delay(500);
}

void loop()
{
    rampMotor(motorA, "MotorA");
    rampMotor(motorB, "MotorB");
    delay(1000);
}
