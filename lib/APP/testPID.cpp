#include "testPID.h"

#include "SuperCar.h"
#include "UprightPID.h"
#include <Arduino.h>
#include <SimpleFOC.h>

extern UprightPID_t upRightPIDConfig;
extern PIDController Velocity_PID;
extern BLDCMotor motor_A;
extern BLDCMotor motor_B;
extern bool velocityLoopEnabled;
extern float targetAngleOffset;
extern float targetAngleLimit;
extern float targetVelocityLimit;
extern float velocityFeedbackSign;
extern float directSpeedDampingKp;
extern float directSpeedDampingLimit;
extern float directSpeedDampingDeadband;
extern float directSpeedDampingFilterAlpha;
extern float driveMaxVelocity;
extern float driveMaxSteer;
extern float driveVelRampStep;
extern float driveBrakeRampStep;
extern float driveSteerRampStep;
extern float bleStartRampMs;
extern float bleStartMinScale;
extern CarControl_t carCTRL;

static WebServer *pidServer = nullptr;

static float argFloat(const char *name, float currentValue, float minValue, float maxValue)
{
    if (!pidServer->hasArg(name))
    {
        return currentValue;
    }

    String value = pidServer->arg(name);
    value.trim();
    if (value.length() == 0)
    {
        return currentValue;
    }

    return constrain(value.toFloat(), minValue, maxValue);
}

static void applyUprightLimit(float limit)
{
    upRightPIDConfig.Kp_Min = -limit;
    upRightPIDConfig.Kp_Max = limit;
    upRightPIDConfig.Ki_Min = -limit / 2.0f;
    upRightPIDConfig.Ki_Max = limit / 2.0f;
    upRightPIDConfig.Kd_Min = -limit / 2.0f;
    upRightPIDConfig.Kd_Max = limit / 2.0f;
    upRightPIDConfig.outMin = -limit;
    upRightPIDConfig.outMax = limit;
}

static const char *runStateName(uint8_t state)
{
    switch (state)
    {
    case CAR_STATE_BALANCING:
        return "BALANCING";
    case CAR_STATE_SUSPENDED:
        return "SUSPENDED";
    case CAR_STATE_FALLEN:
        return "FALLEN";
    default:
        return "WAIT_STABLE";
    }
}

static String pidJson()
{
    String json = "{";
    json += "\"upright_kp\":" + String(upRightPIDConfig.Kp, 6);
    json += ",\"upright_ki\":" + String(upRightPIDConfig.Ki, 6);
    json += ",\"upright_kd\":" + String(upRightPIDConfig.Kd, 6);
    json += ",\"upright_limit\":" + String(upRightPIDConfig.outMax, 6);
    json += ",\"motor_vel_kp\":" + String(motor_A.PID_velocity.P, 6);
    json += ",\"motor_vel_ki\":" + String(motor_A.PID_velocity.I, 6);
    json += ",\"motor_lpf\":" + String(motor_A.LPF_velocity.Tf, 6);
    json += ",\"voltage_limit\":" + String(motor_A.voltage_limit, 6);
    json += ",\"velocity_kp\":" + String(Velocity_PID.P, 6);
    json += ",\"velocity_ki\":" + String(Velocity_PID.I, 6);
    json += ",\"velocity_kd\":" + String(Velocity_PID.D, 6);
    json += ",\"velocity_loop\":" + String(velocityLoopEnabled ? 1 : 0);
    json += ",\"target_angle_offset\":" + String(targetAngleOffset, 6);
    json += ",\"target_angle_limit\":" + String(targetAngleLimit, 6);
    json += ",\"target_velocity_limit\":" + String(targetVelocityLimit, 6);
    json += ",\"velocity_feedback_sign\":" + String(velocityFeedbackSign, 0);
    json += ",\"direct_speed_damping_kp\":" + String(directSpeedDampingKp, 6);
    json += ",\"direct_speed_damping_limit\":" + String(directSpeedDampingLimit, 6);
    json += ",\"direct_speed_damping_deadband\":" + String(directSpeedDampingDeadband, 6);
    json += ",\"direct_speed_damping_filter_alpha\":" + String(directSpeedDampingFilterAlpha, 6);
    json += ",\"drive_max_velocity\":" + String(driveMaxVelocity, 6);
    json += ",\"drive_max_steer\":" + String(driveMaxSteer, 6);
    json += ",\"drive_vel_ramp_step\":" + String(driveVelRampStep, 6);
    json += ",\"drive_brake_ramp_step\":" + String(driveBrakeRampStep, 6);
    json += ",\"drive_steer_ramp_step\":" + String(driveSteerRampStep, 6);
    json += ",\"ble_start_ramp_ms\":" + String(bleStartRampMs, 0);
    json += ",\"ble_start_min_scale\":" + String(bleStartMinScale, 6);
    json += ",\"angle_x\":" + String(carCTRL.MPUangleX, 6);
    json += ",\"gyro_x\":" + String(carCTRL.MPUgyroX, 6);
    json += ",\"acc_norm\":" + String(carCTRL.MPUaccNorm, 6);
    json += ",\"acc_delta\":" + String(carCTRL.MPUaccDelta, 6);
    json += ",\"raw_car_velocity\":" + String(carCTRL.CarVelocity, 6);
    json += ",\"car_velocity\":" + String(carCTRL.ForwardVelocity, 6);
    json += ",\"forward_velocity\":" + String(carCTRL.ForwardVelocity, 6);
    json += ",\"target_velocity\":" + String(carCTRL.TargetVelocity, 6);
    json += ",\"velocity_error\":" + String(carCTRL.VelocityError, 6);
    json += ",\"target_angle\":" + String(carCTRL.TargetAngle, 6);
    json += ",\"motor_velocity\":" + String(carCTRL.MotorVelocity, 6);
    json += ",\"speed_damping\":" + String(carCTRL.SpeedDamping, 6);
    json += ",\"run_state\":" + String(carCTRL.RunState);
    json += ",\"run_state_name\":\"" + String(runStateName(carCTRL.RunState)) + "\"";
    json += ",\"balance_enabled\":" + String(carCTRL.BalanceEnabled ? 1 : 0);
    json += "}";
    return json;
}

static void sendPidJson()
{
    pidServer->send(200, "application/json", pidJson());
}

static void setPidHandler()
{
    upRightPIDConfig.Kp = argFloat("upright_kp", upRightPIDConfig.Kp, 0.0f, 5.0f);
    upRightPIDConfig.Ki = argFloat("upright_ki", upRightPIDConfig.Ki, 0.0f, 0.1f);
    upRightPIDConfig.Kd = argFloat("upright_kd", upRightPIDConfig.Kd, 0.0f, 0.2f);
    applyUprightLimit(argFloat("upright_limit", upRightPIDConfig.outMax, 5.0f, 200.0f));

    motor_A.PID_velocity.P = argFloat("motor_vel_kp", motor_A.PID_velocity.P, 0.0f, 1.0f);
    motor_B.PID_velocity.P = motor_A.PID_velocity.P;
    motor_A.PID_velocity.I = argFloat("motor_vel_ki", motor_A.PID_velocity.I, 0.0f, 10.0f);
    motor_B.PID_velocity.I = motor_A.PID_velocity.I;
    motor_A.LPF_velocity.Tf = argFloat("motor_lpf", motor_A.LPF_velocity.Tf, 0.0f, 0.2f);
    motor_B.LPF_velocity.Tf = motor_A.LPF_velocity.Tf;

    motor_A.voltage_limit = argFloat("voltage_limit", motor_A.voltage_limit, 0.2f, 4.0f);
    motor_B.voltage_limit = motor_A.voltage_limit;

    Velocity_PID.P = argFloat("velocity_kp", Velocity_PID.P, 0.0f, 5.0f);
    Velocity_PID.I = argFloat("velocity_ki", Velocity_PID.I, 0.0f, 1.0f);
    Velocity_PID.D = argFloat("velocity_kd", Velocity_PID.D, 0.0f, 1.0f);

    if (pidServer->hasArg("velocity_loop"))
    {
        velocityLoopEnabled = pidServer->arg("velocity_loop").toInt() != 0;
    }

    targetAngleOffset = argFloat("target_angle_offset", targetAngleOffset, -20.0f, 20.0f);
    targetAngleLimit = argFloat("target_angle_limit", targetAngleLimit, 2.0f, 25.0f);
    targetVelocityLimit = argFloat("target_velocity_limit", targetVelocityLimit, 4.0f, 35.0f);
    velocityFeedbackSign = argFloat("velocity_feedback_sign", velocityFeedbackSign, -1.0f, 1.0f) >= 0.0f ? 1.0f : -1.0f;
    directSpeedDampingKp = argFloat("direct_speed_damping_kp", directSpeedDampingKp, 0.0f, 2.0f);
    directSpeedDampingLimit = argFloat("direct_speed_damping_limit", directSpeedDampingLimit, 0.0f, 40.0f);
    directSpeedDampingDeadband = argFloat("direct_speed_damping_deadband", directSpeedDampingDeadband, 0.0f, 5.0f);
    directSpeedDampingFilterAlpha =
        argFloat("direct_speed_damping_filter_alpha", directSpeedDampingFilterAlpha, 0.02f, 1.0f);

    driveMaxVelocity = argFloat("drive_max_velocity", driveMaxVelocity, 4.0f, 35.0f);
    driveMaxSteer = argFloat("drive_max_steer", driveMaxSteer, 2.0f, 30.0f);
    driveVelRampStep = argFloat("drive_vel_ramp_step", driveVelRampStep, 0.02f, 1.0f);
    driveBrakeRampStep = argFloat("drive_brake_ramp_step", driveBrakeRampStep, 0.02f, 2.0f);
    driveSteerRampStep = argFloat("drive_steer_ramp_step", driveSteerRampStep, 0.02f, 1.5f);
    bleStartRampMs = argFloat("ble_start_ramp_ms", bleStartRampMs, 200.0f, 4000.0f);
    bleStartMinScale = argFloat("ble_start_min_scale", bleStartMinScale, 0.05f, 1.0f);

    upRightPIDConfig.Ki_Out = 0.0f;

    Serial.println("PID tuning updated:");
    Serial.println(pidJson());
    sendPidJson();
}

void registerPIDTestRoutes(WebServer &server)
{
    pidServer = &server;
    server.on("/pid", HTTP_GET, sendPidJson);
    server.on("/pid/set", HTTP_GET, setPidHandler);
    server.on("/pid/set", HTTP_POST, setPidHandler);
}
