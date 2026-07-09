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
#ifndef Upright_h
#define Upright_h

typedef struct upRightPID
{
    volatile float Kp;
    volatile float Ki;
    volatile float Kd;
    volatile float Limit = 60.0f;

    volatile float Ki_Out = 0.0f;
    volatile float Kp_Min = -Limit;
    volatile float Kp_Max = Limit;

    volatile float Ki_Min = -Limit;
    volatile float Ki_Max = Limit;

    volatile float Kd_Min = -Limit;
    volatile float Kd_Max = Limit;

    volatile float outMin = -Limit;
    volatile float outMax = Limit;

    volatile float PID_Out = 0.0f;

} UprightPID_t;

void UprightPID_Init(UprightPID_t *pidConf, float Kp, float Ki, float Kd, float limit);

float UprightPID(UprightPID_t *pidConf, float targetAngleX, float angleX, float GyroX);

#endif