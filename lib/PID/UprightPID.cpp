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

#include "UprightPID.h"

void UprightPID_Init(UprightPID_t *pidConf, float Kp, float Ki, float Kd, float limit)
{
    pidConf->Kp = Kp;
    pidConf->Ki = Ki;
    pidConf->Kd = Kd;
    pidConf->Ki_Out = 0.0f;
    pidConf->PID_Out = 0.0f;

    pidConf->Kp_Min = -limit;
    pidConf->Kp_Max = limit;

    pidConf->Ki_Min = -limit / 2.0f;
    pidConf->Ki_Max = limit / 2.0f;

    pidConf->Kd_Min = -limit / 2.0f;
    pidConf->Kd_Max = limit / 2.0f;

    pidConf->outMin = -limit;
    pidConf->outMax = limit;
}
float UprightPID(UprightPID_t *pidConf, float targetAngleX, float angleX, float GyroX)
{
    float Error_value;

    float P_Out;
    float D_Out;

    Error_value = angleX + targetAngleX;

    P_Out = Error_value * pidConf->Kp;
    pidConf->Ki_Out += Error_value * pidConf->Ki;
    D_Out = GyroX * pidConf->Kd;

    if (P_Out < pidConf->Kp_Min)
        P_Out = pidConf->Kp_Min;
    else if (P_Out > pidConf->Kp_Max)
        P_Out = pidConf->Kp_Max;

    if (pidConf->Ki_Out < pidConf->Ki_Min)
        pidConf->Ki_Out = pidConf->Ki_Min;
    else if (pidConf->Ki_Out > pidConf->Ki_Max)
        pidConf->Ki_Out = pidConf->Ki_Max;

    if (D_Out < pidConf->Kd_Min)
        D_Out = pidConf->Kd_Min;
    else if (D_Out > pidConf->Kd_Max)
        D_Out = pidConf->Kd_Max;

    pidConf->PID_Out = P_Out + pidConf->Ki_Out + D_Out;

    if (pidConf->PID_Out > pidConf->outMax)
    {
        pidConf->PID_Out = pidConf->outMax;
    }
    else if (pidConf->PID_Out < pidConf->outMin)
    {
        pidConf->PID_Out = pidConf->outMin;
    }
    return pidConf->PID_Out;
}