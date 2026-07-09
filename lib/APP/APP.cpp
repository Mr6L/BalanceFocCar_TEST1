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

#include "APP.h"
#include "ButtonAndBattery.h"
#include "BuzzerSound.h"
#include "RGBLED.h"
#include "SuperCar.h"
#include "UserConfig.h"
#include "testPID.h"
#include <HTTPClient.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <math.h>

#if !WIFI_AP_MODE
const char *ssid = USER_SSID;
const char *password = USER_PASSWORD;
#endif

AppControl_t appCTRL;

WebServer appServer(80);

#if STATIC_IP_MODE /*静态IP配置*/

// 使用这些宏来初始化IPAddress实例
IPAddress staticIP(STATIC_IP_FIRST_OCTET, STATIC_IP_SECOND_OCTET, STATIC_IP_THIRD_OCTET, STATIC_IP_FOURTH_OCTET);
IPAddress gateway(GATEWAY_FIRST_OCTET, GATEWAY_SECOND_OCTET, GATEWAY_THIRD_OCTET, GATEWAY_FOURTH_OCTET);
IPAddress subnet(SUBNET_FIRST_OCTET, SUBNET_SECOND_OCTET, SUBNET_THIRD_OCTET, SUBNET_FOURTH_OCTET);
#endif

static String activeWiFiSsid = "";
static String staWiFiSsid = "";
static String staWiFiPassword = "";
static String remoteServerUrl = "";
static String deviceId = "";
static int remoteLastCommandSeq = -1;
static unsigned long bleDriveRampStartedMs = 0;
static int bleDriveRampSign = 0;
static unsigned long driveUnsafeSinceMs = 0;
static unsigned long driveBlockedUntilMs = 0;
static String lastDriveAbortReason = "";
float driveMaxVelocity = WIFI_DRIVE_MAX_VELOCITY;
float driveMaxSteer = WIFI_DRIVE_MAX_STEER;
float driveVelRampStep = WIFI_DRIVE_VEL_RAMP_STEP;
float driveBrakeRampStep = WIFI_DRIVE_BRAKE_RAMP_STEP;
float driveSteerRampStep = WIFI_DRIVE_STEER_RAMP_STEP;
float bleStartRampMs = XIAOZHI_BLE_START_RAMP_MS;
float bleStartMinScale = XIAOZHI_BLE_START_MIN_SCALE;

static void stopDrive();
static String driveJson();
static String distanceJson();
static String rgbJson();
static String startDistanceDrive(float meters, float speedRatio);
static void applyDriveTargets(float velocity, float steer, const String &direction);
static void applyDriveRatio(float throttle, float turn, const String &direction);
static void stopVoiceDrive();

static void resetBleDriveRamp()
{
    bleDriveRampStartedMs = 0;
    bleDriveRampSign = 0;
}

static bool isBalanceReadyForDrive()
{
    return carCTRL.BalanceEnabled && carCTRL.RunState == CAR_STATE_BALANCING;
}

static String balanceNotReadyJson()
{
    stopDrive();
    return "{\"error\":\"car is not balancing\",\"run_state\":" + String(carCTRL.RunState) + "}";
}

static bool isVoiceDriveUnsafe()
{
    return !isBalanceReadyForDrive() ||
           fabsf(carCTRL.MPUangleX) > XIAOZHI_VOICE_ABORT_ANGLE ||
           fabsf(carCTRL.MPUgyroX) > XIAOZHI_VOICE_ABORT_GYRO;
}

static bool isVoiceStartSafe()
{
    return isBalanceReadyForDrive() &&
           fabsf(carCTRL.MPUangleX) < XIAOZHI_VOICE_START_ANGLE &&
           fabsf(carCTRL.MPUgyroX) < XIAOZHI_VOICE_START_GYRO;
}

static bool isManualDriveCommandActive()
{
    return appCTRL.DriveEnabled && appCTRL.Direction != "stop" && appCTRL.Direction != "mpu";
}

static bool isManualDriveUnsafe(const char **reason)
{
    if (!isManualDriveCommandActive() || !isBalanceReadyForDrive())
    {
        return false;
    }

    if (fabsf(carCTRL.MPUangleX) > DRIVE_ABORT_ANGLE)
    {
        if (reason)
        {
            *reason = "angle";
        }
        return true;
    }

    if (fabsf(carCTRL.MPUgyroX) > DRIVE_ABORT_GYRO)
    {
        if (reason)
        {
            *reason = "gyro";
        }
        return true;
    }

#if DRIVE_OVERSPEED_ABORT_ENABLE
    if (fabsf(carCTRL.ForwardVelocity) > DRIVE_ABORT_FORWARD_VELOCITY)
    {
        if (reason)
        {
            *reason = "overspeed";
        }
        return true;
    }
#endif

    return false;
}

static bool isDriveBlocked()
{
    return (long)(driveBlockedUntilMs - millis()) > 0;
}

static void abortManualDriveForSafety(const char *reason)
{
    stopDrive();
    driveUnsafeSinceMs = 0;
    driveBlockedUntilMs = millis() + DRIVE_ABORT_BLOCK_MS;
    lastDriveAbortReason = reason ? reason : "unstable";

    Serial.print("[DriveSafety] abort manual drive: ");
    Serial.print(lastDriveAbortReason);
    Serial.print(", angle=");
    Serial.print(carCTRL.MPUangleX, 2);
    Serial.print(", gyro=");
    Serial.print(carCTRL.MPUgyroX, 2);
    Serial.print(", car_velocity=");
    Serial.print(carCTRL.CarVelocity, 2);
    Serial.print(", forward_velocity=");
    Serial.println(carCTRL.ForwardVelocity, 2);
}

static String normalizeServerUrl(String url)
{
    url.trim();
    while (url.endsWith("/"))
    {
        url.remove(url.length() - 1);
    }
    return url;
}

static String buildApSsid()
{
    String apSsid = WIFI_AP_SSID_PREFIX;
#if WIFI_AP_APPEND_CHIP_ID
    const uint32_t chipId = (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF);
    char suffix[8];
    snprintf(suffix, sizeof(suffix), "-%06X", chipId);
    apSsid += suffix;
#endif
    return apSsid;
}

static void loadDeviceConfig()
{
    deviceId = buildApSsid();

    Preferences prefs;
    prefs.begin("balancecar", true);
    staWiFiSsid = prefs.getString("ssid", USER_SSID);
    staWiFiPassword = prefs.getString("pass", USER_PASSWORD);
    remoteServerUrl = normalizeServerUrl(prefs.getString("server", REMOTE_SERVER_URL));
    deviceId = prefs.getString("device", deviceId);
    appCTRL.DistanceForwardCalibration =
        constrain(prefs.getFloat("dist_fcal", DISTANCE_FORWARD_CALIBRATION), 0.2f, 5.0f);
    appCTRL.DistanceBackwardCalibration =
        constrain(prefs.getFloat("dist_bcal", DISTANCE_BACKWARD_CALIBRATION), 0.2f, 5.0f);
    prefs.end();
}

static void saveDeviceConfig(const String &ssidValue, const String &passwordValue, const String &serverValue,
                             const String &deviceValue)
{
    Preferences prefs;
    prefs.begin("balancecar", false);
    prefs.putString("ssid", ssidValue);
    prefs.putString("pass", passwordValue);
    prefs.putString("server", normalizeServerUrl(serverValue));
    prefs.putString("device", deviceValue.length() > 0 ? deviceValue : buildApSsid());
    prefs.end();
}

static void clearDeviceConfig()
{
    Preferences prefs;
    prefs.begin("balancecar", false);
    prefs.clear();
    prefs.end();
}

static void connectStaIfConfigured()
{
    if (staWiFiSsid.length() == 0)
    {
        Serial.println("STA WiFi not configured.");
        return;
    }

    Serial.print("Connecting STA WiFi: ");
    Serial.println(staWiFiSsid);
    WiFi.begin(staWiFiSsid.c_str(), staWiFiPassword.c_str());

    const unsigned long startMs = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startMs < WIFI_STA_CONNECT_TIMEOUT_MS)
    {
        delay(300);
        Serial.print(".");
    }
    Serial.println("");

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print("STA connected. IP: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("STA connect timeout. AP config portal remains available.");
    }
}

static void startAppWiFi()
{
    loadDeviceConfig();

#if WIFI_AP_MODE
    WiFi.mode(staWiFiSsid.length() > 0 ? WIFI_AP_STA : WIFI_AP);
    WiFi.setSleep(false);

    IPAddress apIP(WIFI_AP_IP_FIRST_OCTET, WIFI_AP_IP_SECOND_OCTET, WIFI_AP_IP_THIRD_OCTET, WIFI_AP_IP_FOURTH_OCTET);
    IPAddress apGateway(WIFI_AP_IP_FIRST_OCTET, WIFI_AP_IP_SECOND_OCTET, WIFI_AP_IP_THIRD_OCTET, WIFI_AP_IP_FOURTH_OCTET);
    IPAddress apSubnet(255, 255, 255, 0);

    WiFi.softAPConfig(apIP, apGateway, apSubnet);
    activeWiFiSsid = buildApSsid();
    const bool apStarted = WiFi.softAP(activeWiFiSsid.c_str(), WIFI_AP_PASSWORD, WIFI_AP_CHANNEL, WIFI_AP_HIDDEN, WIFI_AP_MAX_CLIENTS);

    Serial.println(" ");
    Serial.println("********************************************************");
    Serial.println(apStarted ? "WiFi AP started" : "WiFi AP start failed");
    Serial.print("SSID: ");
    Serial.println(activeWiFiSsid);
    Serial.print("Password: ");
    Serial.println(WIFI_AP_PASSWORD);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("Device ID: ");
    Serial.println(deviceId);
    Serial.print("Remote server: ");
    Serial.println(remoteServerUrl.length() > 0 ? remoteServerUrl : "(not configured)");
    Serial.println("********************************************************");

    connectStaIfConfigured();
#else
#if STATIC_IP_MODE
    if (WiFi.config(staticIP, gateway, subnet) == false)
    {
        Serial.println(" Static IP Config Failed.");
    }
#endif

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    if (staWiFiSsid.length() == 0)
    {
        staWiFiSsid = ssid;
        staWiFiPassword = password;
    }
    WiFi.begin(staWiFiSsid.c_str(), staWiFiPassword.c_str());
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    activeWiFiSsid = ssid;
    Serial.println(" ");
    Serial.println("********************************************************");
    Serial.println(WiFi.localIP());
    Serial.println("********************************************************");
#endif
}

static void AppServerTask(void *pvParameters)
{
    Serial.begin(115200);
    startAppWiFi();

#if 0
#if STATIC_IP_MODE /*静态IP配置*/
    if (WiFi.config(staticIP, gateway, subnet) == false)
    {
        Serial.println(" Static IP Config Failed.");
    }
#endif

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    // #if !STATIC_IP_MODE
    Serial.println(" ");
    Serial.println("********************************************************");
    Serial.println(WiFi.localIP());
    Serial.println("********************************************************");
    // #endif

#endif

    appServer.on("/connect", appConnectHandler);
    appServer.on("/wifi/status", HTTP_GET, appWiFiStatusHandler);
    appServer.on("/wifi/config", HTTP_GET, appWiFiConfigHandler);
    appServer.on("/wifi/config", HTTP_POST, appWiFiConfigHandler);
    appServer.on("/wifi/clear", HTTP_GET, appWiFiClearHandler);
    appServer.on("/wifi/clear", HTTP_POST, appWiFiClearHandler);
    appServer.on("/voltage", appVoltageHandler);
    appServer.on("/battery", HTTP_GET, appBatteryStatusHandler);
    appServer.on("/mpuset", appMPUsetHandler);
    appServer.on("/move", appMoveHandler);
    appServer.on("/drive", HTTP_GET, appDriveHandler);
    appServer.on("/drive", HTTP_POST, appDriveHandler);
    appServer.on("/control", HTTP_GET, appDriveHandler);
    appServer.on("/control", HTTP_POST, appDriveHandler);
    appServer.on("/drive/status", HTTP_GET, appDriveStatusHandler);
    appServer.on("/distance", HTTP_GET, appDistanceHandler);
    appServer.on("/distance", HTTP_POST, appDistanceHandler);
    appServer.on("/distance/config", HTTP_GET, appDistanceConfigHandler);
    appServer.on("/distance/config", HTTP_POST, appDistanceConfigHandler);
    appServer.on("/distance/stop", HTTP_GET, appDistanceStopHandler);
    appServer.on("/distance/stop", HTTP_POST, appDistanceStopHandler);
    appServer.on("/distance/status", HTTP_GET, appDistanceStatusHandler);
    appServer.on("/poweroff", appPowerOffHandler);
    appServer.on("/rgb", HTTP_GET, appRGBChangeHandler);
    appServer.on("/rgb", HTTP_POST, appRGBChangeHandler);
    appServer.on("/rgb/status", HTTP_GET, appRGBStatusHandler);
    registerPIDTestRoutes(appServer);
    appServer.begin();
    Serial.println("Balance Car AppServer Started");

    for (;;)
    {
        appServer.handleClient();
        vTaskDelay(10);
    }
};

static float approachValue(float current, float target, float step)
{
    if (current < target)
    {
        current += step;
        return current > target ? target : current;
    }
    if (current > target)
    {
        current -= step;
        return current < target ? target : current;
    }
    return current;
}

static bool isVelocityBraking(float current, float target)
{
    if (fabsf(current) < 0.001f)
    {
        return false;
    }

    if (fabsf(target) < 0.001f)
    {
        return true;
    }

    if ((current > 0.0f && target < 0.0f) || (current < 0.0f && target > 0.0f))
    {
        return true;
    }

    return fabsf(target) < fabsf(current);
}

static float fixedDriveRatio(float value, float fixedMagnitude)
{
    if (fabsf(value) < 0.001f)
    {
        return 0.0f;
    }
    return value > 0.0f ? fixedMagnitude : -fixedMagnitude;
}

static float governedDriveVelocityTarget(float target)
{
#if DRIVE_SPEED_GOVERNOR_ENABLE
    if (!isManualDriveCommandActive() || fabsf(target) < 0.001f)
    {
        return target;
    }

    const float forwardVelocity = carCTRL.ForwardVelocity;
    if (target > 0.0f && forwardVelocity > target + DRIVE_SPEED_GOVERNOR_MARGIN)
    {
        const float overspeed = forwardVelocity - target;
        return constrain(target - overspeed * DRIVE_SPEED_GOVERNOR_BRAKE_RATIO, -driveMaxVelocity, target);
    }
    if (target < 0.0f && forwardVelocity < target - DRIVE_SPEED_GOVERNOR_MARGIN)
    {
        const float overspeed = target - forwardVelocity;
        return constrain(target + overspeed * DRIVE_SPEED_GOVERNOR_BRAKE_RATIO, target, driveMaxVelocity);
    }
#endif
    return target;
}

static float distanceMetersFromStart()
{
    const float deltaA = carCTRL.MA_Angle - appCTRL.DistanceStartA;
    const float deltaB = carCTRL.MB_Angle - appCTRL.DistanceStartB;
    const float forwardAngle =
        (MOTOR_A_BALANCE_SIGN * deltaA + MOTOR_B_BALANCE_SIGN * deltaB) / 2.0f;
    const float calibration =
        appCTRL.DistanceTargetMeters >= 0.0f ? appCTRL.DistanceForwardCalibration : appCTRL.DistanceBackwardCalibration;
    return forwardAngle * WHEEL_RADIUS_M * calibration;
}

static void stopDistanceDrive(bool markDone)
{
    appCTRL.DistanceEnabled = false;
    appCTRL.DistanceDone = markDone;
    appCTRL.TimedDriveEnabled = false;
    appCTRL.VoiceDriveEnabled = false;
    appCTRL.Direction = "stop";
    appCTRL.DriveEnabled = false;
    appCTRL.VelocityTarget = appCTRL.MPUOffset;
    appCTRL.SteerTarget = 0.0f;
}

static void updateDistanceDrive()
{
    appCTRL.DistanceMeters = distanceMetersFromStart();
    const float targetAbs = fabsf(appCTRL.DistanceTargetMeters);
    const float traveledAbs = fabsf(appCTRL.DistanceMeters);

    if (traveledAbs >= targetAbs)
    {
        stopDistanceDrive(true);
        return;
    }

    const float remaining = targetAbs - traveledAbs;
    const float direction = appCTRL.DistanceTargetMeters >= 0.0f ? 1.0f : -1.0f;
    float speed = fabsf(appCTRL.DistanceSpeedTarget);
    const unsigned long elapsedMs = millis() - appCTRL.DistanceStartedMs;

    if (elapsedMs < DISTANCE_START_RAMP_MS)
    {
        speed *= constrain((float)elapsedMs / (float)DISTANCE_START_RAMP_MS, 0.25f, 1.0f);
    }

    if (remaining < 0.25f)
    {
        speed *= constrain(remaining / 0.25f, 0.25f, 1.0f);
    }

    appCTRL.Direction = "distance";
    appCTRL.DriveEnabled = true;
    appCTRL.LastDriveCommandMs = millis();
    appCTRL.VelocityTarget = direction * speed;
    appCTRL.SteerTarget = 0.0f;
}

static void CarBrakeTask(void *pvParameters)
{
    for (;;)
    {
        const unsigned long now = millis();
        if (appCTRL.DistanceEnabled)
        {
            updateDistanceDrive();
        }

        if (appCTRL.VoiceDriveEnabled)
        {
            if ((long)(now - appCTRL.VoiceDriveStopMs) >= 0 || isVoiceDriveUnsafe())
            {
                if (isVoiceDriveUnsafe())
                {
                    Serial.println("[XiaoZhiVoice] abort voice drive: unstable");
                }
                stopVoiceDrive();
            }
            else
            {
                appCTRL.Direction = "voice";
                appCTRL.DriveEnabled = true;
                appCTRL.LastDriveCommandMs = now;
                appCTRL.VelocityTarget = appCTRL.VoiceVelocityTarget;
                appCTRL.SteerTarget = appCTRL.VoiceSteerTarget;
            }
        }

        if (appCTRL.TimedDriveEnabled)
        {
            if ((long)(now - appCTRL.TimedDriveStopMs) >= 0)
            {
                appCTRL.TimedDriveEnabled = false;
                appCTRL.DistanceEnabled = false;
                appCTRL.DistanceDone = false;
                appCTRL.DriveEnabled = false;
                appCTRL.Direction = "stop";
                appCTRL.VelocityTarget = appCTRL.MPUOffset;
                appCTRL.SteerTarget = 0.0f;
            }
            else
            {
                appCTRL.LastDriveCommandMs = now;
            }
        }

        if (appCTRL.DriveEnabled && now - appCTRL.LastDriveCommandMs > WIFI_DRIVE_TIMEOUT_MS)
        {
            appCTRL.DriveEnabled = false;
            appCTRL.Direction = "stop";
            appCTRL.VelocityTarget = appCTRL.MPUOffset;
            appCTRL.SteerTarget = 0.0f;
        }

#if DRIVE_UNSTABLE_ABORT_ENABLE
        const char *driveUnsafeReason = nullptr;
        if (isManualDriveUnsafe(&driveUnsafeReason))
        {
            if (driveUnsafeSinceMs == 0)
            {
                driveUnsafeSinceMs = now;
            }
            else if (now - driveUnsafeSinceMs >= DRIVE_ABORT_CONFIRM_MS)
            {
                abortManualDriveForSafety(driveUnsafeReason);
            }
        }
        else
        {
            driveUnsafeSinceMs = 0;
        }
#endif

        if (appCTRL.Direction == "stop")
        {
            appCTRL.VelocityTarget = appCTRL.MPUOffset;
            appCTRL.SteerTarget = 0.0f;
        }
        else if (appCTRL.Direction == "mpu")
        {
            appCTRL.VelocityTarget = appCTRL.MPUOffset;
            appCTRL.SteerTarget = 0.0f;
        }

        const float governedVelocityTarget = governedDriveVelocityTarget(appCTRL.VelocityTarget);
        const float velocityRampStep =
            isVelocityBraking(appCTRL.Velocity, governedVelocityTarget) ? driveBrakeRampStep : driveVelRampStep;
        appCTRL.Velocity = approachValue(appCTRL.Velocity, governedVelocityTarget, velocityRampStep);
        appCTRL.SteerVelocity = approachValue(appCTRL.SteerVelocity, appCTRL.SteerTarget, driveSteerRampStep);
        vTaskDelay(10);
    }
};

static void RemoteClientTask(void *pvParameters);

void AppTaskInit::startTask()
{
    xTaskCreatePinnedToCore(AppServerTask, "App Sever Task", 6144, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(CarBrakeTask, "Brake Car Task", 2048, NULL, 1, NULL, 0);
#if REMOTE_CONTROL_ENABLE
    xTaskCreatePinnedToCore(RemoteClientTask, "Remote Client Task", 8192, NULL, 1, NULL, 0);
#endif
};

/*-------------------------------------------------------------------------*/

static void appConnectHandler()
{
    appServer.send(200, "text/plain", "1");
};

static void appWiFiStatusHandler()
{
    String json = "{";
#if WIFI_AP_MODE
    json += "\"mode\":\"ap_sta\"";
    json += ",\"ap_ssid\":\"" + activeWiFiSsid + "\"";
    json += ",\"ap_ip\":\"" + WiFi.softAPIP().toString() + "\"";
    json += ",\"clients\":" + String(WiFi.softAPgetStationNum());
    json += ",\"sta_ssid\":\"" + staWiFiSsid + "\"";
    json += ",\"sta_connected\":" + String(WiFi.status() == WL_CONNECTED ? 1 : 0);
    json += ",\"sta_ip\":\"" + WiFi.localIP().toString() + "\"";
#else
    json += "\"mode\":\"sta\"";
    json += ",\"ssid\":\"" + staWiFiSsid + "\"";
    json += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
    json += ",\"connected\":" + String(WiFi.status() == WL_CONNECTED ? 1 : 0);
#endif
    json += ",\"device_id\":\"" + deviceId + "\"";
    json += ",\"remote_server\":\"" + remoteServerUrl + "\"";
    json += "}";
    appServer.send(200, "application/json", json);
};

static void appWiFiConfigHandler()
{
    if (appServer.hasArg("ssid") || appServer.hasArg("server"))
    {
        const String ssidValue = appServer.arg("ssid");
        const String passwordValue = appServer.arg("password");
        const String serverValue = appServer.arg("server");
        const String deviceValue = appServer.arg("device");

        saveDeviceConfig(ssidValue, passwordValue, serverValue, deviceValue);
        appServer.send(200, "text/html; charset=utf-8",
                       "<!doctype html><meta charset='utf-8'><body>"
                       "<h2>Saved.</h2><p>The car will restart and connect to the configured WiFi/server.</p>"
                       "<script>setTimeout(()=>location.href='/wifi/status',5000)</script></body>");
        delay(300);
        ESP.restart();
        return;
    }

    String html = "<!doctype html><html><head><meta charset='utf-8'><title>BalanceCar WiFi Config</title>"
                  "<style>body{font-family:Arial;margin:24px;background:#f4f6f8;color:#111827}"
                  "form{background:#fff;border:1px solid #d0d7e2;border-radius:8px;padding:16px;max-width:520px}"
                  "label{display:block;margin-top:12px;font-size:14px}input{width:100%;box-sizing:border-box;padding:10px;margin-top:5px}"
                  "button{margin-top:16px;padding:10px 16px;font-weight:bold}</style></head><body>"
                  "<h1>BalanceCar WiFi Config</h1><form method='POST' action='/wifi/config'>"
                  "<label>WiFi SSID</label><input name='ssid' value='" +
                  staWiFiSsid +
                  "' required>"
                  "<label>WiFi Password</label><input name='password' type='password' value='" +
                  staWiFiPassword +
                  "'>"
                  "<label>Remote server URL</label><input name='server' placeholder='https://example.ngrok-free.app' value='" +
                  remoteServerUrl +
                  "'>"
                  "<label>Device ID</label><input name='device' value='" +
                  deviceId +
                  "'>"
                  "<button type='submit'>Save and restart</button></form>"
                  "<p><a href='/wifi/status'>WiFi status</a> | <a href='/wifi/clear'>Clear saved config</a></p>"
                  "</body></html>";
    appServer.send(200, "text/html; charset=utf-8", html);
};

static void appWiFiClearHandler()
{
    clearDeviceConfig();
    appServer.send(200, "text/html; charset=utf-8",
                   "<!doctype html><meta charset='utf-8'><body><h2>Config cleared.</h2>"
                   "<p>The car will restart in AP-only mode.</p></body>");
    delay(300);
    ESP.restart();
};

static void appVoltageHandler()
{
    appServer.send(200, "text/plain", String(batteryVoltage, 3));
};

static void appBatteryStatusHandler()
{
    appServer.send(200, "application/json", batteryStatusJson());
};

static void appMPUsetHandler()
{
    appCTRL.Direction = appServer.arg("direction");
    appCTRL.MPUOffset = appServer.arg("distance").toFloat();
    appCTRL.VelocityTarget = appCTRL.MPUOffset;
};

static String driveJson()
{
    String json = "{";
    json += "\"direction\":\"" + appCTRL.Direction + "\"";
    json += ",\"drive_enabled\":" + String(appCTRL.DriveEnabled ? 1 : 0);
    json += ",\"velocity\":" + String(appCTRL.Velocity, 6);
    json += ",\"steer\":" + String(appCTRL.SteerVelocity, 6);
    json += ",\"velocity_target\":" + String(appCTRL.VelocityTarget, 6);
    json += ",\"steer_target\":" + String(appCTRL.SteerTarget, 6);
    json += ",\"max_velocity\":" + String(driveMaxVelocity, 6);
    json += ",\"max_steer\":" + String(driveMaxSteer, 6);
    json += ",\"target_velocity_limit\":" + String(targetVelocityLimit, 6);
    json += ",\"fixed_throttle_ratio\":" + String(DRIVE_FIXED_THROTTLE_RATIO, 6);
    json += ",\"fixed_turn_ratio\":" + String(DRIVE_FIXED_TURN_RATIO, 6);
    json += ",\"vel_ramp_step\":" + String(driveVelRampStep, 6);
    json += ",\"brake_ramp_step\":" + String(driveBrakeRampStep, 6);
    json += ",\"steer_ramp_step\":" + String(driveSteerRampStep, 6);
    json += ",\"forward_velocity\":" + String(carCTRL.ForwardVelocity, 6);
    json += ",\"overspeed_limit\":" + String(DRIVE_ABORT_FORWARD_VELOCITY, 6);
    json += ",\"safety_blocked\":" + String(isDriveBlocked() ? 1 : 0);
    json += ",\"last_abort_reason\":\"" + lastDriveAbortReason + "\"";
    json += ",\"timeout_ms\":" + String(WIFI_DRIVE_TIMEOUT_MS);
    json += "}";
    return json;
}

static void appDriveStatusHandler()
{
    appServer.send(200, "application/json", driveJson());
}

static void stopDrive()
{
    resetBleDriveRamp();
    appCTRL.DistanceEnabled = false;
    appCTRL.DistanceDone = false;
    appCTRL.TimedDriveEnabled = false;
    appCTRL.VoiceDriveEnabled = false;
    appCTRL.DriveEnabled = false;
    appCTRL.Direction = "stop";
    appCTRL.VelocityTarget = appCTRL.MPUOffset;
    appCTRL.SteerTarget = 0.0f;
}

static void stopVoiceDrive()
{
    appCTRL.VoiceDriveEnabled = false;
    appCTRL.DriveEnabled = false;
    appCTRL.Direction = "stop";
    appCTRL.VoiceVelocityTarget = 0.0f;
    appCTRL.VoiceSteerTarget = 0.0f;
    appCTRL.VelocityTarget = appCTRL.MPUOffset;
    appCTRL.SteerTarget = 0.0f;
}

static void applyDriveTargets(float velocity, float steer, const String &direction)
{
    if (!isBalanceReadyForDrive() && (fabsf(velocity) > 0.001f || fabsf(steer) > 0.001f))
    {
        stopDrive();
        return;
    }

    if (isDriveBlocked() && (fabsf(velocity) > 0.001f || fabsf(steer) > 0.001f))
    {
        stopDrive();
        return;
    }

    appCTRL.DistanceEnabled = false;
    appCTRL.DistanceDone = false;
    appCTRL.TimedDriveEnabled = false;
    appCTRL.VoiceDriveEnabled = false;
    appCTRL.Direction = direction;
    appCTRL.DriveEnabled = true;
    appCTRL.LastDriveCommandMs = millis();
    appCTRL.VelocityTarget = constrain(velocity, -driveMaxVelocity, driveMaxVelocity);
    appCTRL.SteerTarget = constrain(steer, -driveMaxSteer, driveMaxSteer);
}

static void applyDriveRatio(float throttle, float turn, const String &direction)
{
    applyDriveTargets(
        constrain(throttle, -1.0f, 1.0f) * driveMaxVelocity,
        -constrain(turn, -1.0f, 1.0f) * driveMaxSteer,
        direction);
}

static void appMoveHandler()
{
    appCTRL.DistanceEnabled = false;
    appCTRL.DistanceDone = false;
    appCTRL.TimedDriveEnabled = false;
    const String direction = appServer.arg("direction");
    if (direction != "stop" && !isBalanceReadyForDrive())
    {
        appServer.send(409, "application/json", balanceNotReadyJson());
        return;
    }

    appCTRL.Direction = direction;
    appCTRL.DriveEnabled = true;
    appCTRL.LastDriveCommandMs = millis();

    if (appCTRL.Direction == "up")
    {
        appCTRL.VelocityTarget =
            constrain(appCTRL.VelocityTarget + MOVE_VEL, -driveMaxVelocity, driveMaxVelocity);
    }

    if (appCTRL.Direction == "down")
    {
        appCTRL.VelocityTarget =
            constrain(appCTRL.VelocityTarget - MOVE_VEL, -driveMaxVelocity, driveMaxVelocity);
    }

    if (appCTRL.Direction == "left")
    {
        appCTRL.SteerTarget =
            constrain(appCTRL.SteerTarget - STR_VEL, -driveMaxSteer, driveMaxSteer);
    }

    if (appCTRL.Direction == "right")
    {
        appCTRL.SteerTarget =
            constrain(appCTRL.SteerTarget + STR_VEL, -driveMaxSteer, driveMaxSteer);
    }

    if (appCTRL.Direction == "stop")
    {
        stopDrive();
    }

    appServer.send(200, "application/json", driveJson());
};

static float driveArg(const char *name, float defaultValue)
{
    return appServer.hasArg(name) ? appServer.arg(name).toFloat() : defaultValue;
}

static void appDriveHandler()
{
    const bool stopRequested =
        (appServer.hasArg("enable") && appServer.arg("enable").toInt() == 0) ||
        (appServer.hasArg("stop") && appServer.arg("stop").toInt() != 0) ||
        (appServer.hasArg("brake") && appServer.arg("brake").toInt() != 0);

    if (stopRequested)
    {
        stopDrive();
        appServer.send(200, "application/json", driveJson());
        return;
    }

    float velocity = appCTRL.VelocityTarget;
    float steer = appCTRL.SteerTarget;

    if (appServer.hasArg("throttle"))
    {
        velocity = constrain(appServer.arg("throttle").toFloat(), -1.0f, 1.0f) * driveMaxVelocity;
    }
    else if (appServer.hasArg("velocity"))
    {
        velocity = driveArg("velocity", velocity);
    }
    else if (appServer.hasArg("v"))
    {
        velocity = driveArg("v", velocity);
    }

    if (appServer.hasArg("turn"))
    {
        steer = -constrain(appServer.arg("turn").toFloat(), -1.0f, 1.0f) * driveMaxSteer;
    }
    else if (appServer.hasArg("steer"))
    {
        steer = -driveArg("steer", -steer);
    }
    else if (appServer.hasArg("s"))
    {
        steer = -driveArg("s", -steer);
    }

    applyDriveTargets(velocity, steer, "drive");

    appServer.send(200, "application/json", driveJson());
};

static String distanceJson()
{
    const float remaining =
        appCTRL.DistanceEnabled ? (fabsf(appCTRL.DistanceTargetMeters) - fabsf(appCTRL.DistanceMeters)) : 0.0f;
    String json = "{";
    json += "\"enabled\":" + String(appCTRL.DistanceEnabled ? 1 : 0);
    json += ",\"done\":" + String(appCTRL.DistanceDone ? 1 : 0);
    json += ",\"target_m\":" + String(appCTRL.DistanceTargetMeters, 4);
    json += ",\"traveled_m\":" + String(appCTRL.DistanceMeters, 4);
    json += ",\"remaining_m\":" + String(remaining > 0.0f ? remaining : 0.0f, 4);
    json += ",\"speed_target\":" + String(appCTRL.DistanceSpeedTarget, 4);
    json += ",\"wheel_diameter_m\":" + String(WHEEL_DIAMETER_M, 4);
    json += ",\"wheel_radius_m\":" + String(WHEEL_RADIUS_M, 4);
    json += ",\"forward_calibration\":" + String(appCTRL.DistanceForwardCalibration, 4);
    json += ",\"backward_calibration\":" + String(appCTRL.DistanceBackwardCalibration, 4);
    json += "}";
    return json;
}

static bool updateCalibrationFromArg(const char *primaryName, const char *shortName, volatile float &value)
{
    String argValue = "";
    if (appServer.hasArg(primaryName))
    {
        argValue = appServer.arg(primaryName);
    }
    else if (appServer.hasArg(shortName))
    {
        argValue = appServer.arg(shortName);
    }
    else
    {
        return false;
    }

    value = constrain(argValue.toFloat(), 0.2f, 5.0f);
    return true;
}

static void appDistanceConfigHandler()
{
    const bool forwardUpdated =
        updateCalibrationFromArg("forward_calibration", "forward", appCTRL.DistanceForwardCalibration) ||
        updateCalibrationFromArg("fcal", "fc", appCTRL.DistanceForwardCalibration);
    const bool backwardUpdated =
        updateCalibrationFromArg("backward_calibration", "backward", appCTRL.DistanceBackwardCalibration) ||
        updateCalibrationFromArg("bcal", "bc", appCTRL.DistanceBackwardCalibration);

    if (forwardUpdated || backwardUpdated)
    {
        Preferences prefs;
        prefs.begin("balancecar", false);
        prefs.putFloat("dist_fcal", appCTRL.DistanceForwardCalibration);
        prefs.putFloat("dist_bcal", appCTRL.DistanceBackwardCalibration);
        prefs.end();
    }

    if (appCTRL.DistanceEnabled)
    {
        appCTRL.DistanceMeters = distanceMetersFromStart();
    }
    appServer.send(200, "application/json", distanceJson());
}

static void appDistanceStatusHandler()
{
    if (appCTRL.DistanceEnabled)
    {
        appCTRL.DistanceMeters = distanceMetersFromStart();
    }
    appServer.send(200, "application/json", distanceJson());
}

static void appDistanceStopHandler()
{
    stopDistanceDrive(false);
    appServer.send(200, "application/json", distanceJson());
}

static String startDistanceDrive(float meters, float speedRatio)
{
    if (!isBalanceReadyForDrive())
    {
        return balanceNotReadyJson();
    }

    if (fabsf(meters) < 0.01f)
    {
        return "{\"error\":\"meters must be non-zero\"}";
    }

    meters = constrain(meters, -DISTANCE_MAX_METERS, DISTANCE_MAX_METERS);
    speedRatio = constrain(fabsf(speedRatio), DISTANCE_MIN_SPEED_RATIO, 1.0f);

    appCTRL.DistanceEnabled = true;
    appCTRL.DistanceDone = false;
    appCTRL.TimedDriveEnabled = false;
    appCTRL.DistanceTargetMeters = meters;
    appCTRL.DistanceMeters = 0.0f;
    appCTRL.DistanceSpeedTarget = speedRatio * driveMaxVelocity;
    appCTRL.DistanceStartA = carCTRL.MA_Angle;
    appCTRL.DistanceStartB = carCTRL.MB_Angle;
    appCTRL.DistanceStartedMs = millis();
    appCTRL.Direction = "distance";
    appCTRL.DriveEnabled = true;
    appCTRL.LastDriveCommandMs = millis();
    appCTRL.SteerTarget = 0.0f;

    return distanceJson();
}

static void appDistanceHandler()
{
    const bool stopRequested =
        (appServer.hasArg("enable") && appServer.arg("enable").toInt() == 0) ||
        (appServer.hasArg("stop") && appServer.arg("stop").toInt() != 0);

    if (stopRequested)
    {
        stopDistanceDrive(false);
        appServer.send(200, "application/json", distanceJson());
        return;
    }

    float meters = 0.0f;
    if (appServer.hasArg("meters"))
    {
        meters = appServer.arg("meters").toFloat();
    }
    else if (appServer.hasArg("m"))
    {
        meters = appServer.arg("m").toFloat();
    }

    float speedRatio = DISTANCE_DEFAULT_SPEED_RATIO;
    if (appServer.hasArg("speed"))
    {
        speedRatio = appServer.arg("speed").toFloat();
    }
    else if (appServer.hasArg("throttle"))
    {
        speedRatio = appServer.arg("throttle").toFloat();
    }

    const String json = startDistanceDrive(meters, speedRatio);
    appServer.send(json.indexOf("\"error\"") >= 0 ? 400 : 200, "application/json", json);
};

static bool nextCsvToken(const String &payload, int &cursor, String &token)
{
    if (cursor > payload.length())
    {
        return false;
    }

    const int next = payload.indexOf(',', cursor);
    if (next < 0)
    {
        token = payload.substring(cursor);
        cursor = payload.length() + 1;
    }
    else
    {
        token = payload.substring(cursor, next);
        cursor = next + 1;
    }
    token.trim();
    return true;
}

static int remoteHttpGet(const String &url, String &payload)
{
    payload = "";

    if (url.startsWith("https://"))
    {
        WiFiClientSecure client;
        client.setInsecure();
        HTTPClient http;
        http.setTimeout(REMOTE_HTTP_TIMEOUT_MS);
        if (!http.begin(client, url))
        {
            return -1;
        }
        http.addHeader("ngrok-skip-browser-warning", "true");
        const int code = http.GET();
        if (code > 0)
        {
            payload = http.getString();
        }
        http.end();
        return code;
    }

    WiFiClient client;
    HTTPClient http;
    http.setTimeout(REMOTE_HTTP_TIMEOUT_MS);
    if (!http.begin(client, url))
    {
        return -1;
    }
    http.addHeader("ngrok-skip-browser-warning", "true");
    const int code = http.GET();
    if (code > 0)
    {
        payload = http.getString();
    }
    http.end();
    return code;
}

static String remoteUrl(const String &path)
{
    return remoteServerUrl + path;
}

static void applyRemoteDrive(float throttle, float turn)
{
    applyDriveRatio(throttle, turn, "remote");
}

static void applyRemoteRgb(bool enable, int r, int g, int b)
{
    if (enable)
    {
        appCTRL.RGBStatus = "on";
        appCTRL.Color = "{" + String(constrain(r, 0, 255)) + "," + String(constrain(g, 0, 255)) + "," +
                        String(constrain(b, 0, 255)) + "}";
        rgb.ResetColor(appCTRL.Color);
    }
    else
    {
        appCTRL.RGBStatus = "off";
        appCTRL.FrontLightStatus = "off";
        appCTRL.RearLightStatus = "off";
        turnOffRGB();
    }
}

static void handleRemoteCommand(String payload)
{
    payload.trim();
    if (payload.length() == 0 || payload == "N")
    {
        return;
    }

    int cursor = 0;
    String type;
    String token;
    if (!nextCsvToken(payload, cursor, type))
    {
        return;
    }
    if (!nextCsvToken(payload, cursor, token))
    {
        return;
    }
    const int seq = token.toInt();
    if (seq == remoteLastCommandSeq)
    {
        return;
    }

    if (type == "D")
    {
        String throttleText;
        String turnText;
        if (!nextCsvToken(payload, cursor, throttleText) || !nextCsvToken(payload, cursor, turnText))
        {
            return;
        }
        applyRemoteDrive(throttleText.toFloat(), turnText.toFloat());
        remoteLastCommandSeq = seq;
        return;
    }

    if (type == "S")
    {
        stopDrive();
        remoteLastCommandSeq = seq;
        return;
    }

    if (type == "R")
    {
        String enableText;
        String rText;
        String gText;
        String bText;
        if (!nextCsvToken(payload, cursor, enableText) || !nextCsvToken(payload, cursor, rText) ||
            !nextCsvToken(payload, cursor, gText) || !nextCsvToken(payload, cursor, bText))
        {
            return;
        }
        applyRemoteRgb(enableText.toInt() != 0, rText.toInt(), gText.toInt(), bText.toInt());
        remoteLastCommandSeq = seq;
    }
}

static void pollRemoteCommand()
{
    String payload;
    const String url = remoteUrl("/api/device/poll?device_id=" + deviceId);
    const int code = remoteHttpGet(url, payload);
    if (code == 200)
    {
        handleRemoteCommand(payload);
    }
}

static void uploadRemoteStatus()
{
    String payload;
    String url = remoteUrl("/api/device/status?device_id=" + deviceId);
    url += "&angle_x=" + String(carCTRL.MPUangleX, 4);
    url += "&gyro_x=" + String(carCTRL.MPUgyroX, 4);
    url += "&car_velocity=" + String(carCTRL.CarVelocity, 4);
    url += "&forward_velocity=" + String(carCTRL.ForwardVelocity, 4);
    url += "&target_velocity=" + String(carCTRL.TargetVelocity, 4);
    url += "&run_state=" + String(carCTRL.RunState);
    url += "&battery=" + String(batteryVoltage, 3);
    url += "&battery_percent=" + String(batteryPercent);
    url += "&battery_low=" + String(batteryLow ? 1 : 0);
    url += "&rssi=" + String(WiFi.RSSI());
    remoteHttpGet(url, payload);
}

static void RemoteClientTask(void *pvParameters)
{
    unsigned long lastPollMs = 0;
    unsigned long lastStatusMs = 0;

    for (;;)
    {
        if (remoteServerUrl.length() == 0 || staWiFiSsid.length() == 0)
        {
            vTaskDelay(1000);
            continue;
        }

        if (WiFi.status() != WL_CONNECTED)
        {
            WiFi.reconnect();
            vTaskDelay(1000);
            continue;
        }

        const unsigned long now = millis();
        if (now - lastPollMs >= REMOTE_POLL_INTERVAL_MS)
        {
            pollRemoteCommand();
            lastPollMs = now;
        }
        if (now - lastStatusMs >= REMOTE_STATUS_INTERVAL_MS)
        {
            uploadRemoteStatus();
            lastStatusMs = now;
        }

        vTaskDelay(20);
    }
}

static void appPowerOffHandler()
{
    balanceCarPowerOff();
}

static bool rgbSwitchValue(const String &value, bool defaultValue)
{
    String lower = value;
    lower.trim();
    lower.toLowerCase();

    if (lower == "1" || lower == "on" || lower == "true" || lower == "yes")
    {
        return true;
    }
    if (lower == "0" || lower == "off" || lower == "false" || lower == "no")
    {
        return false;
    }
    return defaultValue;
}

static bool requestedRgbSwitchValue(bool defaultValue)
{
    if (appServer.hasArg("enable"))
    {
        return rgbSwitchValue(appServer.arg("enable"), defaultValue);
    }
    if (appServer.hasArg("status"))
    {
        return rgbSwitchValue(appServer.arg("status"), defaultValue);
    }
    return defaultValue;
}

static String requestedRgbColorOr(const String &fallback)
{
    if (!appServer.hasArg("color"))
    {
        return fallback;
    }

    String color = appServer.arg("color");
    color.trim();
    return color.length() > 0 ? color : fallback;
}

static void setFrontLight(bool enable, const String &color)
{
    if (enable)
    {
        appCTRL.FrontLightStatus = "on";
        appCTRL.FrontLightColor = color;
        rgb.SetFrontColor(color);
    }
    else
    {
        appCTRL.FrontLightStatus = "off";
        rgb.TurnOffFront();
    }
}

static void setRearLight(bool enable, const String &color)
{
    if (enable)
    {
        appCTRL.RearLightStatus = "on";
        appCTRL.RearLightColor = color;
        rgb.SetRearColor(color);
    }
    else
    {
        appCTRL.RearLightStatus = "off";
        rgb.TurnOffRear();
    }
}

static String rgbJson()
{
    String json = "{";
    json += "\"status\":\"" + appCTRL.RGBStatus + "\"";
    json += ",\"color\":\"" + appCTRL.Color + "\"";
    json += ",\"front\":\"" + appCTRL.FrontLightStatus + "\"";
    json += ",\"front_color\":\"" + appCTRL.FrontLightColor + "\"";
    json += ",\"rear\":\"" + appCTRL.RearLightStatus + "\"";
    json += ",\"rear_color\":\"" + appCTRL.RearLightColor + "\"";
    json += "}";
    return json;
}

String BalanceCarStopDrive()
{
    stopDrive();
    return driveJson();
}

String BalanceCarDriveFor(float throttleRatio, float turnRatio, unsigned long durationMs)
{
    if (!isVoiceStartSafe())
    {
        stopDrive();
        return "{\"error\":\"car is not stable enough for BLE drive\",\"run_state\":" + String(carCTRL.RunState) +
               ",\"angle_x\":" + String(carCTRL.MPUangleX, 3) +
               ",\"gyro_x\":" + String(carCTRL.MPUgyroX, 3) + "}";
    }

    if (durationMs < 100)
    {
        durationMs = 100;
    }
    if (durationMs > XIAOZHI_BLE_MAX_TIMED_DRIVE_MS)
    {
        durationMs = XIAOZHI_BLE_MAX_TIMED_DRIVE_MS;
    }

    const float limitedThrottle =
        constrain(throttleRatio, -XIAOZHI_BLE_MANUAL_MAX_THROTTLE, XIAOZHI_BLE_MANUAL_MAX_THROTTLE);
    const float limitedTurn =
        constrain(turnRatio, -XIAOZHI_BLE_MANUAL_MAX_TURN, XIAOZHI_BLE_MANUAL_MAX_TURN);

    const unsigned long now = millis();
    const int throttleSign = limitedThrottle > 0.001f ? 1 : (limitedThrottle < -0.001f ? -1 : 0);
    const bool restartRamp =
        throttleSign != bleDriveRampSign ||
        appCTRL.Direction != "ble" ||
        !appCTRL.DriveEnabled ||
        now - appCTRL.LastDriveCommandMs > (unsigned long)(durationMs + 200);

    if (restartRamp)
    {
        bleDriveRampStartedMs = now;
        bleDriveRampSign = throttleSign;
    }

    float rampScale = 1.0f;
    if (throttleSign != 0)
    {
        const float progress =
            constrain((float)(now - bleDriveRampStartedMs) / bleStartRampMs, 0.0f, 1.0f);
        rampScale = bleStartMinScale + (1.0f - bleStartMinScale) * progress;
    }

    const float velocityTarget = limitedThrottle * driveMaxVelocity * rampScale;
    const float steerTarget = -limitedTurn * driveMaxSteer;

    applyDriveTargets(velocityTarget, steerTarget, "ble");
    appCTRL.TimedDriveEnabled = true;
    appCTRL.TimedDriveStopMs = appCTRL.LastDriveCommandMs + durationMs;

    Serial.print("[BleDrive] velocity_target=");
    Serial.print(velocityTarget, 4);
    Serial.print(" steer_target=");
    Serial.print(steerTarget, 4);
    Serial.print(" raw_throttle=");
    Serial.print(throttleRatio, 4);
    Serial.print(" throttle=");
    Serial.print(limitedThrottle, 4);
    Serial.print(" raw_turn=");
    Serial.print(turnRatio, 4);
    Serial.print(" turn=");
    Serial.print(limitedTurn, 4);
    Serial.print(" ramp=");
    Serial.print(rampScale, 3);
    Serial.print(" duration=");
    Serial.println(durationMs);

    String json = driveJson();
    json.remove(json.length() - 1);
    json += ",\"mode\":\"ble\"";
    json += ",\"raw_throttle\":" + String(throttleRatio, 4);
    json += ",\"throttle\":" + String(limitedThrottle, 4);
    json += ",\"raw_turn\":" + String(turnRatio, 4);
    json += ",\"turn\":" + String(limitedTurn, 4);
    json += ",\"ramp_scale\":" + String(rampScale, 3);
    json += ",\"duration_ms\":" + String(durationMs);
    json += "}";
    return json;
}

String BalanceCarMoveDistance(float meters, float speedRatio)
{
    if (!isBalanceReadyForDrive())
    {
        return balanceNotReadyJson();
    }

    speedRatio = constrain(fabsf(speedRatio), DISTANCE_MIN_SPEED_RATIO, XIAOZHI_BLE_MAX_MOVE_SPEED_RATIO);
    return startDistanceDrive(meters, speedRatio);
}

String BalanceCarTurnInPlace(float degrees, float speedRatio)
{
    if (!isBalanceReadyForDrive())
    {
        return balanceNotReadyJson();
    }

    if (fabsf(degrees) < 1.0f)
    {
        return "{\"error\":\"degrees must be non-zero\"}";
    }

    speedRatio = constrain(fabsf(speedRatio), 0.15f, 1.0f);
    const unsigned long durationMs = (unsigned long)constrain(
        fabsf(degrees) / 180.0f * XIAOZHI_TURN_180_MS,
        150.0f,
        (float)XIAOZHI_BLE_MAX_TIMED_DRIVE_MS);

    applyDriveTargets(
        appCTRL.MPUOffset,
        (degrees >= 0.0f ? -speedRatio : speedRatio) * driveMaxSteer,
        "drive");
    appCTRL.TimedDriveEnabled = true;
    appCTRL.TimedDriveStopMs = appCTRL.LastDriveCommandMs + durationMs;

    String json = driveJson();
    json.remove(json.length() - 1);
    json += ",\"degrees\":" + String(degrees, 2);
    json += ",\"duration_ms\":" + String(durationMs);
    json += "}";
    return json;
}

String BalanceCarSetRgb(bool enable, int r, int g, int b)
{
    if (enable)
    {
        appCTRL.RGBStatus = "on";
        appCTRL.Color = "{" + String(constrain(r, 0, 255)) + "," + String(constrain(g, 0, 255)) + "," +
                        String(constrain(b, 0, 255)) + "}";
        rgb.ResetColor(appCTRL.Color);
    }
    else
    {
        appCTRL.RGBStatus = "off";
        appCTRL.FrontLightStatus = "off";
        appCTRL.RearLightStatus = "off";
        turnOffRGB();
    }
    return rgbJson();
}

String BalanceCarSetFrontLight(bool enable, int r, int g, int b)
{
    String color = "{" + String(constrain(r, 0, 255)) + "," + String(constrain(g, 0, 255)) + "," +
                   String(constrain(b, 0, 255)) + "}";
    setFrontLight(enable, color);
    return rgbJson();
}

String BalanceCarSetRearLight(bool enable, int r, int g, int b)
{
    String color = "{" + String(constrain(r, 0, 255)) + "," + String(constrain(g, 0, 255)) + "," +
                   String(constrain(b, 0, 255)) + "}";
    setRearLight(enable, color);
    return rgbJson();
}

String BalanceCarControlStatusJson()
{
    if (appCTRL.DistanceEnabled)
    {
        appCTRL.DistanceMeters = distanceMetersFromStart();
    }

    String json = "{";
    json += "\"direction\":\"" + appCTRL.Direction + "\"";
    json += ",\"drive\":" + String(appCTRL.DriveEnabled ? 1 : 0);
    json += ",\"timed\":" + String(appCTRL.TimedDriveEnabled ? 1 : 0);
    json += ",\"voice\":" + String(appCTRL.VoiceDriveEnabled ? 1 : 0);
    json += ",\"voice_velocity\":" + String(appCTRL.VoiceVelocityTarget, 4);
    json += ",\"voice_steer\":" + String(appCTRL.VoiceSteerTarget, 4);
    json += ",\"distance\":" + String(appCTRL.DistanceEnabled ? 1 : 0);
    json += ",\"distance_done\":" + String(appCTRL.DistanceDone ? 1 : 0);
    json += ",\"target_m\":" + String(appCTRL.DistanceTargetMeters, 3);
    json += ",\"traveled_m\":" + String(appCTRL.DistanceMeters, 3);
    json += ",\"rgb\":\"" + appCTRL.RGBStatus + "\"";
    json += ",\"color\":\"" + appCTRL.Color + "\"";
    json += ",\"front\":\"" + appCTRL.FrontLightStatus + "\"";
    json += ",\"rear\":\"" + appCTRL.RearLightStatus + "\"";
    json += ",\"battery\":" + String(batteryVoltage, 3);
    json += ",\"battery_percent\":" + String(batteryPercent);
    json += ",\"battery_low\":" + String(batteryLow ? 1 : 0);
    json += ",\"battery_critical\":" + String(batteryCritical ? 1 : 0);
    json += ",\"battery_adc_raw\":" + String(batteryAdcRaw);
    json += ",\"battery_adc_mv\":" + String(batteryAdcMilliVolts);
    json += ",\"target_velocity\":" + String(carCTRL.TargetVelocity, 4);
    json += ",\"forward_velocity\":" + String(carCTRL.ForwardVelocity, 4);
    json += ",\"velocity_error\":" + String(carCTRL.VelocityError, 4);
    json += ",\"drive_safety_blocked\":" + String(isDriveBlocked() ? 1 : 0);
    json += ",\"drive_abort_reason\":\"" + lastDriveAbortReason + "\"";
    json += ",\"run_state\":" + String(carCTRL.RunState);
    json += ",\"balance\":" + String(carCTRL.BalanceEnabled ? 1 : 0);
    json += "}";
    return json;
}

static void appRGBStatusHandler()
{
    appServer.send(200, "application/json", rgbJson());
}

static void appRGBChangeHandler()
{
    bool handledFrontBackLight = false;

    if (appServer.hasArg("front"))
    {
        setFrontLight(
            rgbSwitchValue(appServer.arg("front"), true),
            requestedRgbColorOr(appCTRL.FrontLightColor));
        handledFrontBackLight = true;
    }

    if (appServer.hasArg("rear"))
    {
        setRearLight(
            rgbSwitchValue(appServer.arg("rear"), true),
            requestedRgbColorOr(appCTRL.RearLightColor));
        handledFrontBackLight = true;
    }

    if (appServer.hasArg("target"))
    {
        String target = appServer.arg("target");
        target.toLowerCase();
        const bool enable = requestedRgbSwitchValue(true);

        if (target.indexOf("front") >= 0)
        {
            setFrontLight(enable, requestedRgbColorOr(appCTRL.FrontLightColor));
            handledFrontBackLight = true;
        }
        if (target.indexOf("rear") >= 0 || target.indexOf("back") >= 0)
        {
            setRearLight(enable, requestedRgbColorOr(appCTRL.RearLightColor));
            handledFrontBackLight = true;
        }
    }

    if (handledFrontBackLight)
    {
        buzzer.play(changeColorID);
        appServer.send(200, "application/json", rgbJson());
        return;
    }

    if (appServer.hasArg("color"))
    {
        appCTRL.Color = appServer.arg("color");
    }
    if (appServer.hasArg("status"))
    {
        appCTRL.RGBStatus = appServer.arg("status");
    }
    if (appServer.hasArg("enable"))
    {
        appCTRL.RGBStatus = appServer.arg("enable").toInt() != 0 ? "on" : "off";
    }

    if (appCTRL.RGBStatus == "on")
    {
        buzzer.play(changeColorID);
        rgb.ResetColor(appCTRL.Color);
    }
    else if (appCTRL.RGBStatus == "off")
    {
        appCTRL.FrontLightStatus = "off";
        appCTRL.RearLightStatus = "off";
        turnOffRGB();
    }

    appServer.send(200, "application/json", rgbJson());
}

AppTaskInit APP;
