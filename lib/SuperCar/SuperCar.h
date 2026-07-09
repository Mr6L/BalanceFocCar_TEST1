#ifndef SUPERCAR_H
#define SUPERCAR_H

#include <Arduino.h>
#include "DcMotor.h"
#include "QuadratureEncoder.h"
#include "PID.h"
#include "UprightPID.h"

enum CarRunState : uint8_t
{
    CAR_STATE_WAIT_STABLE = 0,
    CAR_STATE_BALANCING = 1,
    CAR_STATE_SUSPENDED = 2,
    CAR_STATE_FALLEN = 3
};

typedef struct carControl
{
    float MPUangleX = 0.0f;
    float MPUgyroX = 0.0f;
    float MPUaccNorm = 1.0f;
    float MPUaccDelta = 0.0f;

    float MA_Velocity = 0.0f;
    float MB_Velocity = 0.0f;
    float MA_Angle = 0.0f;
    float MB_Angle = 0.0f;

    float CarVelocity = 0.0f;
    float ForwardVelocity = 0.0f;
    float TargetVelocity = 0.0f;
    float SteerVelocity = 0.0f;
    float VelocityError = 0.0f;
    float TargetAngle = 0.0f;
    float MotorVelocity = 0.0f;
    float SpeedDamping = 0.0f;

    bool BalanceEnabled = false;
    uint8_t RunState = CAR_STATE_WAIT_STABLE;
} CarControl_t;

class SuperCar
{
  public:
    void startTask();
    void running();
};

extern SuperCar BalanceCar;
extern CarControl_t carCTRL;

// Motor & encoder instances
extern DcMotor motorA;
extern DcMotor motorB;
extern QuadratureEncoder encoderA;
extern QuadratureEncoder encoderB;

// PID controllers (tunable via WebControl/BLE)
extern UprightPID_t upRightPIDConfig;
extern PID velocityPid;      // outer speed -> tilt angle
extern PID motorVelPidA;     // inner wheel speed -> PWM
extern PID motorVelPidB;

// Tunable parameters exposed to the web/BLE tuning interface
extern float targetAngleOffset;
extern float targetAngleLimit;
extern float targetVelocityLimit;
extern float velocityFeedbackSign;
extern bool velocityLoopEnabled;
extern float motorVelLpfAlpha;
extern float motorPwmLimit;
extern float directSpeedDampingKp;
extern float directSpeedDampingLimit;
extern float directSpeedDampingDeadband;
extern float directSpeedDampingFilterAlpha;

const char *runStateName(uint8_t state);

// Drive helpers shared by BLE and WebControl.
// throttle and turn are normalized to [-1, 1].
String setDriveCommand(float throttle, float turn, unsigned long durationMs);
String stopDriveCommand();
String carStatusJson();

// For boards without a power-latch circuit: disable motors and deep sleep.
void powerOffCar();

#endif
