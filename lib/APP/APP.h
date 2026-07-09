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
#ifndef APP_h
#define APP_h

#include <Arduino.h>
#include <UserConfig.h>

/******************************IP配置***********************************/
#define STATIC_IP_FIRST_OCTET 192
#define STATIC_IP_SECOND_OCTET 168
#define STATIC_IP_THIRD_OCTET YOUR_IP 
#define STATIC_IP_FOURTH_OCTET 80

#define GATEWAY_FIRST_OCTET 192
#define GATEWAY_SECOND_OCTET 168
#define GATEWAY_THIRD_OCTET 221
#define GATEWAY_FOURTH_OCTET 78

#define SUBNET_FIRST_OCTET 255
#define SUBNET_SECOND_OCTET YOUR_IP 
#define SUBNET_THIRD_OCTET 255
#define SUBNET_FOURTH_OCTET 0

/*********************************************************************/

#define BRAKE_DECAY 5.0f
#define MOVE_VEL 4.0f
#define STR_VEL 2.0f

typedef struct appControl
{
    String Direction = "stop";
    volatile float MPUOffset = 0.0f;
    volatile float Velocity = 0.0f;
    volatile float SteerVelocity = 0.0f;
    volatile float VelocityTarget = 0.0f;
    volatile float SteerTarget = 0.0f;
    volatile bool DriveEnabled = false;
    volatile unsigned long LastDriveCommandMs = 0;
    volatile bool TimedDriveEnabled = false;
    volatile unsigned long TimedDriveStopMs = 0;
    volatile bool VoiceDriveEnabled = false;
    volatile float VoiceVelocityTarget = 0.0f;
    volatile float VoiceSteerTarget = 0.0f;
    volatile unsigned long VoiceDriveStopMs = 0;
    volatile bool DistanceEnabled = false;
    volatile bool DistanceDone = false;
    volatile float DistanceTargetMeters = 0.0f;
    volatile float DistanceMeters = 0.0f;
    volatile float DistanceSpeedTarget = 0.0f;
    volatile float DistanceStartA = 0.0f;
    volatile float DistanceStartB = 0.0f;
    volatile float DistanceForwardCalibration = DISTANCE_FORWARD_CALIBRATION;
    volatile float DistanceBackwardCalibration = DISTANCE_BACKWARD_CALIBRATION;
    volatile unsigned long DistanceStartedMs = 0;

    // RGB LED
    String Color = "{0,0,0}";
    String RGBStatus = "off";
    String FrontLightColor = "{255,255,255}";
    String FrontLightStatus = "off";
    String RearLightColor = "{255,0,0}";
    String RearLightStatus = "off";

} AppControl_t;

void AppControlInit(AppControl_t *control);
String BalanceCarStopDrive();
String BalanceCarDriveFor(float throttleRatio, float turnRatio, unsigned long durationMs);
String BalanceCarMoveDistance(float meters, float speedRatio);
String BalanceCarTurnInPlace(float degrees, float speedRatio);
String BalanceCarSetRgb(bool enable, int r, int g, int b);
String BalanceCarSetFrontLight(bool enable, int r, int g, int b);
String BalanceCarSetRearLight(bool enable, int r, int g, int b);
String BalanceCarControlStatusJson();

static void appConnectHandler();
static void appWiFiStatusHandler();
static void appWiFiConfigHandler();
static void appWiFiClearHandler();
static void appVoltageHandler();
static void appBatteryStatusHandler();
static void appMPUsetHandler();
static void appMoveHandler();
static void appDriveHandler();
static void appDriveStatusHandler();
static void appDistanceHandler();
static void appDistanceConfigHandler();
static void appDistanceStopHandler();
static void appDistanceStatusHandler();
static void appRGBChangeHandler();
static void appRGBStatusHandler();
static void appPowerOffHandler();

class AppTaskInit
{
  public:
    void startTask();
};

extern AppTaskInit APP;
extern AppControl_t appCTRL;

#endif
