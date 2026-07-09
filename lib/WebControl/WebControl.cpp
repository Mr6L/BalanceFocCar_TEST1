#include "WebControl.h"

#include "CarCommands.h"
#include "SuperCar.h"
#include "UserConfig.h"

#include <WebServer.h>
#include <WiFi.h>

static WebServer server(80);

static void sendJson(const String &json)
{
    server.send(200, "application/json", json);
}

static void sendOkJson(const String &extra = "")
{
    String json = "{\"ok\":true";
    if (extra.length())
    {
        json += "," + extra;
    }
    json += "}";
    sendJson(json);
}

static IPAddress buildApIp()
{
    return IPAddress(WIFI_AP_IP_FIRST_OCTET,
                     WIFI_AP_IP_SECOND_OCTET,
                     WIFI_AP_IP_THIRD_OCTET,
                     WIFI_AP_IP_FOURTH_OCTET);
}

static void startWiFiAp()
{
    String ssid = WIFI_AP_SSID_PREFIX;
#if WIFI_AP_APPEND_CHIP_ID
    ssid += "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
#endif

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(buildApIp(), buildApIp(), IPAddress(255, 255, 255, 0));
    WiFi.softAP(ssid.c_str(),
                WIFI_AP_PASSWORD,
                WIFI_AP_CHANNEL,
                WIFI_AP_HIDDEN != 0,
                WIFI_AP_MAX_CLIENTS);

    Serial.print("[WebControl] AP IP: ");
    Serial.println(WiFi.softAPIP());
}

static bool parseFloat(const String &value, float &out)
{
    if (value.length() == 0)
    {
        return false;
    }
    out = value.toFloat();
    return true;
}

static void applyPidParameter(const String &key, const String &value)
{
    float newVal = 0.0f;
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;

    if (key == "upright_kp" && parseFloat(value, newVal))
    {
        upRightPIDConfig.Kp = newVal;
    }
    else if (key == "upright_ki" && parseFloat(value, newVal))
    {
        upRightPIDConfig.Ki = newVal;
    }
    else if (key == "upright_kd" && parseFloat(value, newVal))
    {
        upRightPIDConfig.Kd = newVal;
    }
    else if (key == "upright_limit" && parseFloat(value, newVal))
    {
        upRightPIDConfig.Limit = newVal;
        upRightPIDConfig.outMin = -newVal;
        upRightPIDConfig.outMax = newVal;
        upRightPIDConfig.Kp_Min = -newVal;
        upRightPIDConfig.Kp_Max = newVal;
        upRightPIDConfig.Ki_Min = -newVal;
        upRightPIDConfig.Ki_Max = newVal;
        upRightPIDConfig.Kd_Min = -newVal;
        upRightPIDConfig.Kd_Max = newVal;
    }
    else if (key == "velocity_kp" && parseFloat(value, newVal))
    {
        velocityPid.getGains(kp, ki, kd);
        velocityPid.setGains(newVal, ki, kd);
    }
    else if (key == "velocity_ki" && parseFloat(value, newVal))
    {
        velocityPid.getGains(kp, ki, kd);
        velocityPid.setGains(kp, newVal, kd);
    }
    else if (key == "velocity_kd" && parseFloat(value, newVal))
    {
        velocityPid.getGains(kp, ki, kd);
        velocityPid.setGains(kp, ki, newVal);
    }
    else if (key == "velocity_loop")
    {
        velocityLoopEnabled = (value.toInt() != 0);
    }
    else if (key == "motor_vel_kp" && parseFloat(value, newVal))
    {
        motorVelPidA.getGains(kp, ki, kd);
        motorVelPidA.setGains(newVal, ki, kd);
        motorVelPidB.getGains(kp, ki, kd);
        motorVelPidB.setGains(newVal, ki, kd);
    }
    else if (key == "motor_vel_ki" && parseFloat(value, newVal))
    {
        motorVelPidA.getGains(kp, ki, kd);
        motorVelPidA.setGains(kp, newVal, kd);
        motorVelPidB.getGains(kp, ki, kd);
        motorVelPidB.setGains(kp, newVal, kd);
    }
    else if (key == "motor_lpf" && parseFloat(value, newVal))
    {
        motorVelLpfAlpha = newVal;
    }
    else if (key == "voltage_limit" && parseFloat(value, newVal))
    {
        motorPwmLimit = constrain(newVal, 0.0f, 1.0f);
        motorVelPidA.setOutputLimit(motorPwmLimit);
        motorVelPidB.setOutputLimit(motorPwmLimit);
    }
    else if (key == "target_angle_offset" && parseFloat(value, newVal))
    {
        targetAngleOffset = newVal;
    }
    else if (key == "target_angle_limit" && parseFloat(value, newVal))
    {
        targetAngleLimit = newVal;
        velocityPid.setOutputLimit(newVal);
    }
    else if (key == "target_velocity_limit" && parseFloat(value, newVal))
    {
        targetVelocityLimit = newVal;
    }
    else if (key == "velocity_feedback_sign")
    {
        velocityFeedbackSign = (value.toInt() >= 0) ? 1.0f : -1.0f;
    }
    else if (key == "direct_speed_damping_kp" && parseFloat(value, newVal))
    {
        directSpeedDampingKp = newVal;
    }
    else if (key == "direct_speed_damping_limit" && parseFloat(value, newVal))
    {
        directSpeedDampingLimit = newVal;
    }
    else if (key == "direct_speed_damping_deadband" && parseFloat(value, newVal))
    {
        directSpeedDampingDeadband = newVal;
    }
    else if (key == "direct_speed_damping_filter_alpha" && parseFloat(value, newVal))
    {
        directSpeedDampingFilterAlpha = newVal;
    }
}

static void handleRoot()
{
    server.send(200, "text/plain", "BalanceCar WebControl ready");
}

static void handlePid()
{
    sendJson(carStatusJson());
}

static void handlePidSet()
{
    for (int i = 0; i < server.args(); ++i)
    {
        applyPidParameter(server.argName(i), server.arg(i));
    }
    sendJson(carStatusJson());
}

static void handleDrive()
{
    if (server.hasArg("stop"))
    {
        sendJson(stopDriveCommand());
        return;
    }

    float throttle = 0.0f;
    float turn = 0.0f;
    if (server.hasArg("throttle"))
    {
        throttle = server.arg("throttle").toFloat();
    }
    if (server.hasArg("turn"))
    {
        turn = server.arg("turn").toFloat();
    }

    // Keep the command alive for a bit so brief network hiccups don't stop the car.
    const unsigned long durationMs = 1500;
    sendJson(setDriveCommand(throttle, turn, durationMs));
}

static void handleDriveStatus()
{
    sendJson(carStatusJson());
}

static void handleBattery()
{
    // No battery monitoring hardware on this board.
    sendJson("{\"ok\":true,\"voltage\":0,\"percent\":0,\"note\":\"no_battery\"}");
}

static void handleWifiStatus()
{
    String json = "{";
    json += "\"ok\":true";
    json += ",\"clients\":" + String(WiFi.softAPgetStationNum());
    json += ",\"rssi\":" + String(WiFi.RSSI());
    json += "}";
    sendJson(json);
}

static void handleUnsupported()
{
    sendOkJson("\"note\":\"unsupported\"");
}

static void WebControlLoop(void *pvParameters)
{
    for (;;)
    {
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void WebControlTask::startTask()
{
#if WEB_CONTROL_ENABLE
    startWiFiAp();

    server.on("/", HTTP_GET, handleRoot);
    server.on("/pid", HTTP_GET, handlePid);
    server.on("/pid/set", HTTP_GET, handlePidSet);
    server.on("/pid/set", HTTP_POST, handlePidSet);
    server.on("/drive", HTTP_GET, handleDrive);
    server.on("/drive/status", HTTP_GET, handleDriveStatus);
    server.on("/battery", HTTP_GET, handleBattery);
    server.on("/wifi/status", HTTP_GET, handleWifiStatus);

    // No RGB / distance hardware; return polite no-op.
    server.on("/rgb", HTTP_GET, handleUnsupported);
    server.on("/rgb/status", HTTP_GET, handleUnsupported);
    server.on("/distance", HTTP_GET, handleUnsupported);
    server.on("/distance/stop", HTTP_GET, handleUnsupported);
    server.on("/distance/status", HTTP_GET, handleUnsupported);
    server.on("/distance/config", HTTP_GET, handleUnsupported);

    server.begin();
    Serial.println("[WebControl] HTTP server started on port 80");

    xTaskCreatePinnedToCore(WebControlLoop, "WebControl", 4096, NULL, 1, NULL, 0);
#else
    Serial.println("[WebControl] disabled");
#endif
}

WebControlTask WebControl;
