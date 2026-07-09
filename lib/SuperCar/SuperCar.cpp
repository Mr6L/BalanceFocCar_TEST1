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

#include "SuperCar.h"
#include "APP.h"
#include "ButtonAndBattery.h"
#include "UprightPID.h"
#include "UserConfig.h"
#include <Arduino.h>
#include <MPU6050.h>
#include <SimpleFOC.h>
#include <math.h>

// sensor instance
MagneticSensorI2C sensor_A = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor_B = MagneticSensorI2C(AS5600_I2C);

TwoWire I2C_A = TwoWire(1);
TwoWire I2C_B = TwoWire(0);

MPU6050 mpu6050(I2C_B);

// BLDC motor & driver instance
BLDCMotor motor_A = BLDCMotor(7);
BLDCMotor motor_B = BLDCMotor(7);
BLDCDriver3PWM driver_A = BLDCDriver3PWM(MOTOR_A_PWM_U, MOTOR_A_PWM_V, MOTOR_A_PWM_W);
BLDCDriver3PWM driver_B = BLDCDriver3PWM(MOTOR_B_PWM_U, MOTOR_B_PWM_V, MOTOR_B_PWM_W);

// -----------------------------------------------

#if MONITOR_MODE
Commander command = Commander(Serial);
void doMotorA(char *cmd)
{
    command.motor(&motor_A, cmd);
}

void doMotorB(char *cmd)
{
    command.motor(&motor_B, cmd);
}
#endif
// -----------------------------------------------
UprightPID_t upRightPIDConfig;
// PIDController Velocity_PID{.P = VELOCITY_Kp, .I = VELOCITY_Ki, .D = VELOCITY_Kd, .ramp = 1000, .limit =
// VELOCITY_LIMIT};
PIDController Velocity_PID(VELOCITY_Kp, VELOCITY_Ki, VELOCITY_Kd, 1000, VELOCITY_LIMIT);
bool velocityLoopEnabled = ENABLE_VELOCITY_LOOP;
float targetAngleOffset = 1.1f;
float targetAngleLimit = TARGET_ANGLE_LIMIT;
float targetVelocityLimit = TARGET_VEL_LIMIT;
float velocityFeedbackSign = VELOCITY_FEEDBACK_SIGN;
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
//------------------------------------------------

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

static const char *focDirectionName(Direction direction)
{
    switch (direction)
    {
    case Direction::CW:
        return "Direction::CW";
    case Direction::CCW:
        return "Direction::CCW";
    default:
        return "Direction::UNKNOWN";
    }
}

static void printFocCalibration(const char *motorName, BLDCMotor &motor, int initResult)
{
    Serial.print("[FOC] ");
    Serial.print(motorName);
    Serial.print(" init_result=");
    Serial.println(initResult);

    Serial.print("#define ");
    Serial.print(motorName);
    Serial.print("_SENSOR_DIRECTION ");
    Serial.println(focDirectionName(motor.sensor_direction));

    Serial.print("#define ");
    Serial.print(motorName);
    Serial.print("_ZERO_ELECTRIC_ANGLE ");
    Serial.print(motor.zero_electric_angle, 6);
    Serial.println("f");
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
    motor_A.target = 0.0f;
    motor_B.target = 0.0f;
    upRightPIDConfig.Ki_Out = 0.0f;
    Velocity_PID.reset();
    motor_A.PID_velocity.reset();
    motor_B.PID_velocity.reset();
}

static void clearDriveControl()
{
    appCTRL.Direction = "stop";
    appCTRL.Velocity = appCTRL.MPUOffset;
    appCTRL.SteerVelocity = 0.0f;
    appCTRL.VelocityTarget = appCTRL.MPUOffset;
    appCTRL.SteerTarget = 0.0f;
    appCTRL.DriveEnabled = false;
    appCTRL.TimedDriveEnabled = false;
    appCTRL.VoiceDriveEnabled = false;
    appCTRL.DistanceEnabled = false;
    appCTRL.DistanceDone = false;
}

static void disableMotorOutput()
{
    stopMotorOutput();
    if (!motorOutputEnabled)
    {
        return;
    }

    motor_A.disable();
    motor_B.disable();
    motorOutputEnabled = false;
    Serial.println("Motor output disabled.");
}

static void enableMotorOutput()
{
    if (motorOutputEnabled)
    {
        return;
    }

    motor_A.enable();
    motor_B.enable();
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

    // ---------------------------------------------------------------
    Serial.begin(115200);
    // initialise magnetic sensor hardware
    I2C_A.begin(37, 36, 400000UL);
    I2C_B.begin(8, 9, 400000UL);
    sensor_A.init(&I2C_A);
    sensor_B.init(&I2C_B);

    // link the motor to the sensor
    motor_A.linkSensor(&sensor_A);
    motor_B.linkSensor(&sensor_B);

    // driver config
    // power supply voltage [V]
    driver_A.voltage_power_supply = 8.2;
    driver_B.voltage_power_supply = 8.2;
    driver_A.init();
    driver_B.init();
    // link the motor and the driver
    motor_A.linkDriver(&driver_A);
    motor_B.linkDriver(&driver_B);
    // motor config
    motor_A.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_B.foc_modulation = FOCModulationType::SpaceVectorPWM;
    // set motion control loop to be used
    motor_A.torque_controller = TorqueControlType::voltage;
    motor_B.torque_controller = TorqueControlType::voltage;
    motor_A.controller = MotionControlType::velocity;
    motor_B.controller = MotionControlType::velocity;

    // velocity PI controller parameters
    motor_A.PID_velocity.P = VEL_Kp;
    motor_A.PID_velocity.I = VEL_Ki;
    motor_A.PID_velocity.output_ramp = 1000;
    motor_A.LPF_velocity.Tf = 0.02f;
    motor_A.voltage_sensor_align = 0.6f;
    motor_A.voltage_limit = MOTOR_VOLTAGE_LIMIT;

    motor_B.PID_velocity.P = VEL_Kp;
    motor_B.PID_velocity.I = VEL_Ki;
    motor_B.PID_velocity.output_ramp = 1000;
    motor_B.LPF_velocity.Tf = 0.02f;
    motor_B.voltage_sensor_align = 0.6f;
    motor_B.voltage_limit = MOTOR_VOLTAGE_LIMIT;

    // --------------------------------------
    motor_A.PID_current_q.P = C_Kp;
    motor_A.PID_current_q.I = C_Ki;
    motor_A.LPF_current_q.Tf = C_LF;
    motor_A.PID_current_q.output_ramp = 1000;

    motor_A.PID_current_d.P = C_Kp;
    motor_A.PID_current_d.I = C_Ki;
    motor_A.LPF_current_d.Tf = C_LF;
    motor_A.PID_current_d.output_ramp = 1000;

    motor_A.PID_current_d.limit = 3.0f;
    motor_A.PID_current_q.limit = 3.0f;

    motor_A.velocity_limit = 100;
    motor_A.current_limit = 6.0f;
    // ----------------------------------------
    motor_B.PID_current_q.P = C_Kp;
    motor_B.PID_current_q.I = C_Ki;
    motor_B.LPF_current_q.Tf = C_LF;
    motor_B.PID_current_q.output_ramp = 1000;

    motor_B.PID_current_d.P = C_Kp;
    motor_B.PID_current_d.I = C_Ki;
    motor_B.LPF_current_d.Tf = C_LF;
    motor_B.PID_current_d.output_ramp = 1000;

    motor_B.PID_current_d.limit = 3.0f;
    motor_B.PID_current_q.limit = 3.0f;

    motor_B.velocity_limit = 100;
    motor_B.current_limit = 6.0f;

    // initialize motor
    motor_A.init();
    motor_B.init();

    // Keep current sense disabled in the full car firmware. On Arduino-ESP32 2.x
    // the SimpleFOC MCPWM current-sense ISR can crash during WiFi/NVS flash access.

#if STARTUP_SKIP_FOC_ALIGN
    motor_A.sensor_direction = MOTOR_A_SENSOR_DIRECTION;
    motor_A.zero_electric_angle = MOTOR_A_ZERO_ELECTRIC_ANGLE;
    motor_B.sensor_direction = MOTOR_B_SENSOR_DIRECTION;
    motor_B.zero_electric_angle = MOTOR_B_ZERO_ELECTRIC_ANGLE;
#endif

    const int motorAInitResult = motor_A.initFOC();
    const int motorBInitResult = motor_B.initFOC();
    motorOutputEnabled = motorAInitResult && motorBInitResult;

    Serial.println("[FOC] Calibration values below are valid only when init_result=1.");
    printFocCalibration("MOTOR_A", motor_A, motorAInitResult);
    printFocCalibration("MOTOR_B", motor_B, motorBInitResult);

    // -------------------------------------------------------------------------------------
    UprightPID_Init(&upRightPIDConfig, UPRIGHT_Kp, UPRIGHT_Ki, UPRIGHT_Kd, UPRIGHT_LIMIT);

#if MONITOR_MODE
    command.add('A', doMotorA, "motorA");
    command.add('B', doMotorB, "motorB");
#endif
    mpu6050.begin();
    mpu6050.calcGyroOffsets(false, 500, 0);
    disableMotorOutput();
    balanceArmed = false;
    startupReadyAfterMs = millis() + STARTUP_BALANCE_DELAY_MS;
    landingContactNeeded = false;
    landingContactSeen = true;
    setRunState(CAR_STATE_WAIT_STABLE);
    startPostInitIndicators();
}

CarControl_t carCTRL;
uint8_t velocityCount = 0;
uint8_t angleCount = 0;

void SuperCar::running()
{
    motor_A.loopFOC();
    motor_B.loopFOC();
    if (!motorOutputEnabled)
    {
        motor_A.move(0.0f);
        motor_B.move(0.0f);
    }

    mpu6050.update();
    carCTRL.MPUangleX = mpu6050.getAngleX();
    carCTRL.MPUgyroX = mpu6050.getGyroXFV();
    carCTRL.MPUaccNorm =
        sqrtf(mpu6050.getAccX() * mpu6050.getAccX() + mpu6050.getAccY() * mpu6050.getAccY() +
              mpu6050.getAccZ() * mpu6050.getAccZ());
    carCTRL.MPUaccDelta = fabsf(carCTRL.MPUaccNorm - lastAccNorm);
    lastAccNorm = carCTRL.MPUaccNorm;
    carCTRL.MA_Velocity = motor_A.shaft_velocity;
    carCTRL.MB_Velocity = motor_B.shaft_velocity;
    carCTRL.MA_Angle = motor_A.shaft_angle;
    carCTRL.MB_Angle = motor_B.shaft_angle;
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

#if MONITOR_MODE
    motor_A.monitor();
    motor_B.monitor();
#endif

    if (++velocityCount == VEL_PID_UPDATE)
    {
        if (velocityLoopEnabled)
        {
            carCTRL.TargetVelocity = constrain(appCTRL.Velocity, -targetVelocityLimit, targetVelocityLimit);
            carCTRL.VelocityError = carCTRL.TargetVelocity - carCTRL.ForwardVelocity;
            const float rawTargetAngle = Velocity_PID(carCTRL.VelocityError) + targetAngleOffset;
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
    if (++angleCount == UPRIGHT_PID_UPDATE)
    {
        carCTRL.SteerVelocity = constrain(appCTRL.SteerVelocity, -STR_LIMIT, STR_LIMIT);

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

    motor_A.target = MOTOR_A_BALANCE_SIGN * carCTRL.MotorVelocity + MOTOR_A_STEER_SIGN * carCTRL.SteerVelocity;
    motor_B.target = MOTOR_B_BALANCE_SIGN * carCTRL.MotorVelocity + MOTOR_B_STEER_SIGN * carCTRL.SteerVelocity;
    motor_A.move(motor_A.target);
    motor_B.move(motor_B.target);
}

SuperCar BalanceCar;
