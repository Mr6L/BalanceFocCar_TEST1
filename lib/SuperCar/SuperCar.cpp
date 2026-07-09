/*
  Balance controller for ESP32-S3 + MG513P20_12V DC motors + TB6612.
*/

#include "SuperCar.h"
#include "CarCommands.h"
#include "UprightPID.h"
#include "UserConfig.h"
#include <Arduino.h>
#include <MPU6050.h>
#include <esp_sleep.h>
#include <math.h>

TwoWire I2C_B = TwoWire(0);
MPU6050 mpu6050(I2C_B);

DcMotor motorA;
DcMotor motorB;
QuadratureEncoder encoderA;
QuadratureEncoder encoderB;

UprightPID_t upRightPIDConfig;
float motorPwmLimit = MOTOR_PWM_LIMIT;

PID velocityPid(VELOCITY_Kp, VELOCITY_Ki, VELOCITY_Kd,
                TARGET_ANGLE_LIMIT, TARGET_ANGLE_LIMIT);
PID motorVelPidA(MOTOR_VEL_Kp, MOTOR_VEL_Ki, 0.0f,
                 motorPwmLimit, motorPwmLimit);
PID motorVelPidB(MOTOR_VEL_Kp, MOTOR_VEL_Ki, 0.0f,
                 motorPwmLimit, motorPwmLimit);

bool velocityLoopEnabled = ENABLE_VELOCITY_LOOP != 0;
float targetAngleOffset = 1.0f;
float targetAngleLimit = TARGET_ANGLE_LIMIT;
float targetVelocityLimit = TARGET_VEL_LIMIT;
float velocityFeedbackSign = VELOCITY_FEEDBACK_SIGN;
float motorVelLpfAlpha = MOTOR_VEL_LPF_ALPHA;
float directSpeedDampingKp = DIRECT_SPEED_DAMPING_Kp;
float directSpeedDampingLimit = DIRECT_SPEED_DAMPING_LIMIT;
float directSpeedDampingDeadband = DIRECT_SPEED_DAMPING_DEADBAND;
float directSpeedDampingFilterAlpha = DIRECT_SPEED_DAMPING_FILTER_ALPHA;

static bool balanceArmed = false;
static unsigned long startupReadyAfterMs = 0;
static unsigned long suspendCandidateSinceMs = 0;
static unsigned long landingStableSinceMs = 0;
static unsigned long rearmAllowedAfterMs = 0;
static uint8_t lastPrintedRunState = 255;
static bool landingContactNeeded = false;
static bool landingContactSeen = false;
static float lastAccNorm = 1.0f;
static bool motorOutputEnabled = false;

static uint32_t lastInnerPidUs = 0;
static uint32_t lastOuterPidUs = 0;

static float filteredMA_Velocity = 0.0f;
static float filteredMB_Velocity = 0.0f;

CarControl_t carCTRL;

const char *runStateName(uint8_t state)
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

static void setRunState(uint8_t state)
{
    carCTRL.RunState = state;
    carCTRL.BalanceEnabled = state == CAR_STATE_BALANCING;
    if (lastPrintedRunState != state)
    {
        Serial.print("Balance state: ");
        Serial.println(runStateName(state));
        lastPrintedRunState = state;
    }
}

static void stopMotorOutput()
{
    carCTRL.TargetVelocity = 0.0f;
    carCTRL.SteerVelocity = 0.0f;
    carCTRL.VelocityError = 0.0f;
    carCTRL.TargetAngle = targetAngleOffset;
    carCTRL.MotorVelocity = 0.0f;
    carCTRL.SpeedDamping = 0.0f;

    upRightPIDConfig.Ki_Out = 0.0f;
    velocityPid.reset();
    motorVelPidA.reset();
    motorVelPidB.reset();

    motorA.brake();
    motorB.brake();
}

static void clearDriveControl()
{
    driveCommand.velocity = 0.0f;
    driveCommand.steer = 0.0f;
    driveCommand.enabled = false;
    driveCommand.timeoutMs = 0;
}

static void disableMotorOutput()
{
    stopMotorOutput();
    if (!motorOutputEnabled)
    {
        return;
    }

    motorA.coast();
    motorB.coast();
    motorOutputEnabled = false;
    Serial.println("Motor output disabled.");
}

static void enableMotorOutput()
{
    if (motorOutputEnabled)
    {
        return;
    }

    motorOutputEnabled = true;
    stopMotorOutput();
    Serial.println("Motor output enabled.");
}

static void enterSafeHold(uint8_t state)
{
    clearDriveControl();
    disableMotorOutput();
    balanceArmed = false;
    landingStableSinceMs = 0;
    rearmAllowedAfterMs = millis() + SUSPEND_REARM_DELAY_MS;
    landingContactNeeded = LAND_CONTACT_REQUIRED != 0;
    landingContactSeen = !landingContactNeeded;
    setRunState(state);
}

static bool isSuspendCandidate()
{
    const bool wheelFreeSpin = fabsf(carCTRL.CarVelocity) > SUSPEND_WHEEL_VELOCITY &&
                               fabsf(carCTRL.TargetVelocity) < SUSPEND_TARGET_VELOCITY;
    const bool accAbnormal = carCTRL.MPUaccNorm < SUSPEND_ACC_MIN || carCTRL.MPUaccNorm > SUSPEND_ACC_MAX;
    return wheelFreeSpin || accAbnormal;
}

static bool isLandingReady()
{
    const float maxWheelVelocity = max(fabsf(carCTRL.MA_Velocity), fabsf(carCTRL.MB_Velocity));
    return fabsf(carCTRL.MPUangleX) < LAND_READY_ANGLE && fabsf(carCTRL.MPUgyroX) < LAND_READY_GYRO &&
           maxWheelVelocity < LAND_READY_WHEEL_VELOCITY && carCTRL.MPUaccNorm > LAND_READY_ACC_MIN &&
           carCTRL.MPUaccNorm < LAND_READY_ACC_MAX;
}

static bool isLandingContactDetected()
{
    return carCTRL.MPUaccNorm < LAND_CONTACT_ACC_MIN || carCTRL.MPUaccNorm > LAND_CONTACT_ACC_MAX ||
           carCTRL.MPUaccDelta > LAND_CONTACT_ACC_DELTA;
}

void SuperCar::startTask()
{
    // I2C for MPU6050 only (pins 8/9)
    I2C_B.begin(8, 9, 400000UL);

    // DC motors: PWM + direction pins, sign aligns forward motion
    motorA.init(MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, 0, MOTOR_A_BALANCE_SIGN);
    motorB.init(MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, 1, MOTOR_B_BALANCE_SIGN);

    // Quadrature encoders (output-shaft counts)
    const float countsPerRev = (float)MOTOR_ENCODER_PPR * 4.0f * MOTOR_GEAR_RATIO;
    encoderA.init(MOTOR_A_ENC_A, MOTOR_A_ENC_B, countsPerRev, MOTOR_A_BALANCE_SIGN);
    encoderB.init(MOTOR_B_ENC_A, MOTOR_B_ENC_B, countsPerRev, MOTOR_B_BALANCE_SIGN);

    UprightPID_Init(&upRightPIDConfig, UPRIGHT_Kp, UPRIGHT_Ki, UPRIGHT_Kd, UPRIGHT_LIMIT);

    mpu6050.begin();
    mpu6050.calcGyroOffsets(false, 500, 0);

    disableMotorOutput();
    balanceArmed = false;
    startupReadyAfterMs = millis() + STARTUP_BALANCE_DELAY_MS;
    landingContactNeeded = false;
    landingContactSeen = true;
    setRunState(CAR_STATE_WAIT_STABLE);
}

static float readFilteredVelocity(QuadratureEncoder &encoder, float &filtered, float alpha)
{
    const float raw = encoder.getVelocity();
    filtered += (raw - filtered) * constrain(alpha, 0.0f, 1.0f);
    return filtered;
}

uint8_t velocityCount = 0;
uint8_t angleCount = 0;

void SuperCar::running()
{
    mpu6050.update();
    carCTRL.MPUangleX = mpu6050.getAngleX();
    carCTRL.MPUgyroX = mpu6050.getGyroXFV();
    carCTRL.MPUaccNorm =
        sqrtf(mpu6050.getAccX() * mpu6050.getAccX() + mpu6050.getAccY() * mpu6050.getAccY() +
              mpu6050.getAccZ() * mpu6050.getAccZ());
    carCTRL.MPUaccDelta = fabsf(carCTRL.MPUaccNorm - lastAccNorm);
    lastAccNorm = carCTRL.MPUaccNorm;

    carCTRL.MA_Velocity = readFilteredVelocity(encoderA, filteredMA_Velocity, motorVelLpfAlpha);
    carCTRL.MB_Velocity = readFilteredVelocity(encoderB, filteredMB_Velocity, motorVelLpfAlpha);
    carCTRL.MA_Angle = encoderA.getAngle();
    carCTRL.MB_Angle = encoderB.getAngle();

    carCTRL.CarVelocity =
        (MOTOR_A_BALANCE_SIGN * carCTRL.MA_Velocity + MOTOR_B_BALANCE_SIGN * carCTRL.MB_Velocity) / 2.0f;
    carCTRL.ForwardVelocity = -velocityFeedbackSign * carCTRL.CarVelocity;

#if FALL_PROTECT_ENABLE
    if (fabsf(carCTRL.MPUangleX) > FALL_PROTECT_ANGLE)
    {
        enterSafeHold(CAR_STATE_FALLEN);
        return;
    }
#endif

#if AIR_DETECT_ENABLE
    if (balanceArmed && isSuspendCandidate())
    {
        if (suspendCandidateSinceMs == 0)
        {
            suspendCandidateSinceMs = millis();
        }

        if (millis() - suspendCandidateSinceMs >= SUSPEND_CONFIRM_MS)
        {
            enterSafeHold(CAR_STATE_SUSPENDED);
            return;
        }
    }
    else
    {
        suspendCandidateSinceMs = 0;
    }
#endif

    if (!balanceArmed)
    {
        const bool startupDelayDone = millis() >= startupReadyAfterMs;
        const bool rearmDelayDone = millis() >= rearmAllowedAfterMs;
        if (landingContactNeeded && rearmDelayDone && isLandingContactDetected())
        {
            landingContactSeen = true;
            landingStableSinceMs = 0;
            Serial.println("Landing contact detected.");
        }
        const bool contactReady = !landingContactNeeded || landingContactSeen;
        const bool postureReady = contactReady && isLandingReady();

        disableMotorOutput();

        if (!startupDelayDone || !rearmDelayDone || !postureReady)
        {
            if (carCTRL.RunState != CAR_STATE_SUSPENDED && carCTRL.RunState != CAR_STATE_FALLEN)
            {
                setRunState(CAR_STATE_WAIT_STABLE);
            }
            landingStableSinceMs = 0;
            return;
        }

        if (landingStableSinceMs == 0)
        {
            landingStableSinceMs = millis();
            return;
        }

        if (millis() - landingStableSinceMs < LAND_STABLE_MS)
        {
            return;
        }

        balanceArmed = true;
        enableMotorOutput();
        suspendCandidateSinceMs = 0;
        landingStableSinceMs = 0;
        velocityCount = 0;
        angleCount = 0;
        setRunState(CAR_STATE_BALANCING);
        Serial.println("Balance control armed.");
    }

    // ---- Outer velocity loop ----
    if (++velocityCount == VEL_PID_UPDATE)
    {
        const uint32_t nowUs = micros();
        const float outerDt = (nowUs - lastOuterPidUs) / 1e6f;
        lastOuterPidUs = nowUs;

        // Drive command timeout check
        if (driveCommand.enabled && millis() > driveCommand.timeoutMs)
        {
            driveCommand.enabled = false;
            driveCommand.velocity = 0.0f;
            driveCommand.steer = 0.0f;
        }

        static float rampedVelocity = 0.0f;
        static float rampedSteer = 0.0f;

        float requestedVelocity = 0.0f;
        float requestedSteer = 0.0f;
        if (driveCommand.enabled)
        {
            requestedVelocity = driveCommand.velocity * DRIVE_MAX_VELOCITY;
            requestedSteer = driveCommand.steer * DRIVE_MAX_STEER;
        }

        rampedVelocity += constrain(requestedVelocity - rampedVelocity,
                                    -DRIVE_VEL_RAMP_STEP, DRIVE_VEL_RAMP_STEP);
        rampedSteer += constrain(requestedSteer - rampedSteer,
                                 -DRIVE_STEER_RAMP_STEP, DRIVE_STEER_RAMP_STEP);

        carCTRL.TargetVelocity = constrain(rampedVelocity, -targetVelocityLimit, targetVelocityLimit);
        carCTRL.SteerVelocity = constrain(rampedSteer, -STR_LIMIT, STR_LIMIT);

        if (velocityLoopEnabled)
        {
            carCTRL.VelocityError = carCTRL.TargetVelocity - carCTRL.ForwardVelocity;
            const float rawTargetAngle = velocityPid.update(carCTRL.VelocityError, outerDt) + targetAngleOffset;
            carCTRL.TargetAngle =
                constrain(rawTargetAngle, targetAngleOffset - targetAngleLimit, targetAngleOffset + targetAngleLimit);
        }
        else
        {
            carCTRL.TargetVelocity = 0.0f;
            carCTRL.VelocityError = 0.0f;
            carCTRL.TargetAngle = targetAngleOffset;
        }
        velocityCount = 0;
    }

    // ---- Upright loop ----
    if (++angleCount == UPRIGHT_PID_UPDATE)
    {
        carCTRL.MotorVelocity = UprightPID(&upRightPIDConfig, carCTRL.TargetAngle, carCTRL.MPUangleX, carCTRL.MPUgyroX);

#if DIRECT_SPEED_DAMPING_ENABLE
        const float dampingError = carCTRL.TargetVelocity - carCTRL.ForwardVelocity;
        const float rawSpeedDamping =
            fabsf(dampingError) > directSpeedDampingDeadband
                ? constrain(dampingError * directSpeedDampingKp, -directSpeedDampingLimit, directSpeedDampingLimit)
                : 0.0f;
        carCTRL.SpeedDamping += (rawSpeedDamping - carCTRL.SpeedDamping) *
                                constrain(directSpeedDampingFilterAlpha, 0.0f, 1.0f);
        carCTRL.MotorVelocity = constrain(carCTRL.MotorVelocity + carCTRL.SpeedDamping,
                                          -upRightPIDConfig.outMax,
                                          upRightPIDConfig.outMax);
#else
        carCTRL.SpeedDamping = 0.0f;
#endif
        angleCount = 0;
    }

    // ---- Inner DC motor velocity loop ----
    const uint32_t nowUs = micros();
    const float innerDt = (nowUs - lastInnerPidUs) / 1e6f;
    lastInnerPidUs = nowUs;

    const float leftTarget = MOTOR_A_BALANCE_SIGN * carCTRL.MotorVelocity +
                             MOTOR_A_STEER_SIGN * carCTRL.SteerVelocity;
    const float rightTarget = MOTOR_B_BALANCE_SIGN * carCTRL.MotorVelocity +
                              MOTOR_B_STEER_SIGN * carCTRL.SteerVelocity;

    const float pwmA = motorVelPidA.update(leftTarget - carCTRL.MA_Velocity, innerDt);
    const float pwmB = motorVelPidB.update(rightTarget - carCTRL.MB_Velocity, innerDt);

    motorA.setSpeed(pwmA);
    motorB.setSpeed(pwmB);
}

SuperCar BalanceCar;

// ---------------------------------------------------------------------------
// Shared drive / status helpers used by BLE and WebControl
// ---------------------------------------------------------------------------

String setDriveCommand(float throttle, float turn, unsigned long durationMs)
{
    throttle = constrain(throttle, -1.0f, 1.0f);
    turn = constrain(turn, -1.0f, 1.0f);

    driveCommand.velocity = throttle;
    driveCommand.steer = turn;
    driveCommand.enabled = true;
    driveCommand.timeoutMs = millis() + durationMs;

    String json = "{";
    json += "\"ok\":true";
    json += ",\"cmd\":\"drive\"";
    json += ",\"throttle\":" + String(throttle, 3);
    json += ",\"turn\":" + String(turn, 3);
    json += ",\"duration_ms\":" + String(durationMs);
    json += "}";
    return json;
}

String stopDriveCommand()
{
    clearDriveControl();

    String json = "{";
    json += "\"ok\":true";
    json += ",\"cmd\":\"stop\"";
    json += "}";
    return json;
}

String carStatusJson()
{
    String json = "{";
    json += "\"ok\":true";
    json += ",\"run_state_name\":\"" + String(runStateName(carCTRL.RunState)) + "\"";
    json += ",\"angle_x\":" + String(carCTRL.MPUangleX, 4);
    json += ",\"target_angle\":" + String(carCTRL.TargetAngle, 4);
    json += ",\"target_velocity\":" + String(carCTRL.TargetVelocity, 4);
    json += ",\"car_velocity\":" + String(carCTRL.ForwardVelocity, 4);
    json += ",\"velocity_error\":" + String(carCTRL.VelocityError, 4);
    json += ",\"speed_damping\":" + String(carCTRL.SpeedDamping, 4);
    json += ",\"ma_velocity\":" + String(carCTRL.MA_Velocity, 4);
    json += ",\"mb_velocity\":" + String(carCTRL.MB_Velocity, 4);

    // PID parameters for the tuning panel
    json += ",\"upright_kp\":" + String(upRightPIDConfig.Kp, 4);
    json += ",\"upright_ki\":" + String(upRightPIDConfig.Ki, 4);
    json += ",\"upright_kd\":" + String(upRightPIDConfig.Kd, 4);
    json += ",\"upright_limit\":" + String(upRightPIDConfig.Limit, 4);

    float vkp, vki, vkd;
    velocityPid.getGains(vkp, vki, vkd);
    json += ",\"velocity_kp\":" + String(vkp, 4);
    json += ",\"velocity_ki\":" + String(vki, 4);
    json += ",\"velocity_kd\":" + String(vkd, 4);
    json += ",\"velocity_loop\":" + String(velocityLoopEnabled ? 1 : 0);

    float mkpA, mkiA, mkdA;
    motorVelPidA.getGains(mkpA, mkiA, mkdA);
    json += ",\"motor_vel_kp\":" + String(mkpA, 4);
    json += ",\"motor_vel_ki\":" + String(mkiA, 4);
    json += ",\"motor_lpf\":" + String(motorVelLpfAlpha, 4);
    json += ",\"voltage_limit\":" + String(MOTOR_PWM_LIMIT, 2);

    json += ",\"target_angle_offset\":" + String(targetAngleOffset, 4);
    json += ",\"target_angle_limit\":" + String(targetAngleLimit, 4);
    json += ",\"target_velocity_limit\":" + String(targetVelocityLimit, 4);
    json += ",\"velocity_feedback_sign\":" + String(velocityFeedbackSign >= 0.0f ? 1 : -1);

    json += ",\"direct_speed_damping_kp\":" + String(directSpeedDampingKp, 4);
    json += ",\"direct_speed_damping_limit\":" + String(directSpeedDampingLimit, 4);
    json += ",\"direct_speed_damping_deadband\":" + String(directSpeedDampingDeadband, 4);
    json += ",\"direct_speed_damping_filter_alpha\":" + String(directSpeedDampingFilterAlpha, 4);

    json += ",\"drive_max_velocity\":" + String(DRIVE_MAX_VELOCITY, 4);
    json += ",\"drive_max_steer\":" + String(DRIVE_MAX_STEER, 4);
    json += ",\"drive_vel_ramp_step\":" + String(DRIVE_VEL_RAMP_STEP, 4);
    json += ",\"drive_brake_ramp_step\":" + String(DRIVE_BRAKE_RAMP_STEP, 4);
    json += ",\"drive_steer_ramp_step\":" + String(DRIVE_STEER_RAMP_STEP, 4);
    json += ",\"ble_start_ramp_ms\":" + String(XIAOZHI_BLE_START_RAMP_MS);
    json += ",\"ble_start_min_scale\":" + String(XIAOZHI_BLE_START_MIN_SCALE, 4);

    json += "}";
    return json;
}

// ---------------------------------------------------------------------------
// Power off stub for boards without a power-latch circuit.
// Disables motor output and enters deep sleep until reset.
// ---------------------------------------------------------------------------
void powerOffCar()
{
    Serial.println("Power off requested: disabling motors and entering deep sleep.");
    disableMotorOutput();
    delay(200);
    esp_deep_sleep_start();
}
