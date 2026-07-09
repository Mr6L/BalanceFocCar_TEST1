/*
  ____        _                         _____
 |  _ \      | |                       / ____|
 | |_) | __ _| | __ _ _ __   ___ ___  | |     __ _ _ __
 |  _ < / _` | |/ _` | '_ \ / __/ _ \ | |    / _` | '__|
 | |_) | (_| | | (_| | | | | (_|  __/ | |___| (_| | |
 |____/ \__,_|_|\__,_|_| |_|\___\___|  \_____\__,_|_|
   _____           _           _   ____         __     __  _    _                 _     _
  / ____|         | |         | | |  _ \        \ \   / / | |  | |               | |   | |
 | |     ___  __ _| |_ ___  __| | | |_) |_   _   \ \_/ ___| |__| | __ _ _ __ ___ | | __| |
 | |    / _ \/ _` | __/ _ \/ _` | |  _ <| | | |   \   / _ |  __  |/ _` | '__/ _ \| |/ _` |
 | |___|  __| (_| | ||  __| (_| | | |_) | |_| |    | |  __| |  | | (_| | | | (_) | | (_| |
  \_____\___|\__,_|\__\___|\__,_| |____/ \__, |    |_|\___|_|  |_|\__,_|_|  \___/|_|\__,_|
                                          __/ |
                                         |___/

 Copyright (c) 2024 YeHarold
*/
#ifndef _SUPERCAR_H__
#define _SUPERCAR_H__

#include <Arduino.h>

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
    float TargetVelocity = 1.0f;
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
extern float targetVelocityLimit;

#endif
