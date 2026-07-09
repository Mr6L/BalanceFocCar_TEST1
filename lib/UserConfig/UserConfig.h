/*
  User-configurable constants for the balance car.
  Adapted for ESP32-S3 + MG513P20_12V DC motors + TB6612 + quadrature encoders.
*/
#ifndef UserConfig_h
#define UserConfig_h

/*-------------------- WiFi AP / WebControl --------------------*/
#define WEB_CONTROL_ENABLE 1
#define WIFI_AP_MODE 1
#define WIFI_AP_SSID_PREFIX "BalanceCar"
#define WIFI_AP_PASSWORD "12345678"
#define WIFI_AP_CHANNEL 6
#define WIFI_AP_MAX_CLIENTS 2
#define WIFI_AP_HIDDEN 0
#define WIFI_AP_APPEND_CHIP_ID 1
#define WIFI_AP_IP_FIRST_OCTET 192
#define WIFI_AP_IP_SECOND_OCTET 168
#define WIFI_AP_IP_THIRD_OCTET 4
#define WIFI_AP_IP_FOURTH_OCTET 1

/*-------------------- XiaoZhi BLE control ----------------------*/
#define XIAOZHI_BLE_ENABLE 1
#define XIAOZHI_BLE_DEVICE_NAME "HC-04BLE"
#define XIAOZHI_BLE_SERVICE_UUID "0000FFE0-0000-1000-8000-00805F9B34FB"
#define XIAOZHI_BLE_CHAR_UUID "0000FFE1-0000-1000-8000-00805F9B34FB"
#define XIAOZHI_BLE_MAX_TIMED_DRIVE_MS 4000
#define XIAOZHI_TURN_180_MS 1900
#define XIAOZHI_BLE_DEFAULT_MOVE_SPEED_RATIO 0.16f
#define XIAOZHI_BLE_MAX_MOVE_SPEED_RATIO 0.20f
#define XIAOZHI_BLE_DEFAULT_TURN_RATIO 0.35f
#define XIAOZHI_BLE_MANUAL_MAX_THROTTLE 0.70f
#define XIAOZHI_BLE_MANUAL_MAX_TURN 0.80f
#define XIAOZHI_BLE_START_RAMP_MS 1600
#define XIAOZHI_BLE_START_MIN_SCALE 0.20f

/*-------------------------- Motor hardware ---------------------*/
#define MOTOR_A_PWM 38
#define MOTOR_A_IN1 39
#define MOTOR_A_IN2 40

#define MOTOR_B_PWM 12
#define MOTOR_B_IN1 11
#define MOTOR_B_IN2 10

#define MOTOR_A_ENC_A 1
#define MOTOR_A_ENC_B 5
#define MOTOR_B_ENC_A 2
#define MOTOR_B_ENC_B 6

// MG513P20 encoder: 390 PPR (motor shaft) * 4 (quadrature) * 20 (gearbox)
#define MOTOR_ENCODER_PPR 390
#define MOTOR_GEAR_RATIO 20.0f
#define WHEEL_DIAMETER_M 0.065f
#define WHEEL_RADIUS_M (WHEEL_DIAMETER_M * 0.5f)

// Mechanical direction signs. Calibrate after wiring so that positive
// PWM makes the wheel roll the same physical direction on both sides.
#define MOTOR_A_BALANCE_SIGN 1.0f
#define MOTOR_B_BALANCE_SIGN -1.0f
#define MOTOR_A_STEER_SIGN 1.0f
#define MOTOR_B_STEER_SIGN 1.0f

// PWM output limit (0..1) used by the inner motor-velocity PID output.
#define MOTOR_PWM_LIMIT 1.0f

/*-------------------------- PID Config ------------------------*/
// Outer velocity loop: target speed -> target tilt angle
#define VELOCITY_Kp 1.2f
#define VELOCITY_Ki 0.0f
#define VELOCITY_Kd 0.0f
#define VELOCITY_LIMIT 45.0f
#define ENABLE_VELOCITY_LOOP 1
#define VELOCITY_FEEDBACK_SIGN 1.0f
#define TARGET_ANGLE_LIMIT 12.0f

// Inner DC-motor velocity loop: target wheel speed -> PWM
#define MOTOR_VEL_Kp 0.15f
#define MOTOR_VEL_Ki 0.50f
#define MOTOR_VEL_LPF_ALPHA 0.18f

// Upright loop: target angle + current angle/gyro -> wheel speed
#define UPRIGHT_Kp 1.05f
#define UPRIGHT_Ki 0.0f
#define UPRIGHT_Kd 0.02f
#define UPRIGHT_LIMIT 75.0f

#define TARGET_VEL_LIMIT 30.0f
#define STR_LIMIT 30.0f

#define VEL_PID_UPDATE 4
#define UPRIGHT_PID_UPDATE 2

// Direct speed damping (helps reject disturbances)
#define DIRECT_SPEED_DAMPING_ENABLE 1
#define DIRECT_SPEED_DAMPING_Kp 0.20f
#define DIRECT_SPEED_DAMPING_LIMIT 12.0f
#define DIRECT_SPEED_DAMPING_DEADBAND 0.45f
#define DIRECT_SPEED_DAMPING_FILTER_ALPHA 0.18f

/*------------------------Drive / remote limits------------------*/
#define DRIVE_MAX_VELOCITY 25.0f
#define DRIVE_MAX_STEER 20.0f
#define DRIVE_FIXED_THROTTLE_RATIO 0.30f
#define DRIVE_FIXED_TURN_RATIO 0.20f
#define DRIVE_TIMEOUT_MS 500
#define DRIVE_VEL_RAMP_STEP 0.02f
#define DRIVE_BRAKE_RAMP_STEP 0.20f
#define DRIVE_STEER_RAMP_STEP 0.35f
#define DRIVE_UNSTABLE_ABORT_ENABLE 1
#define DRIVE_ABORT_ANGLE 10.0f
#define DRIVE_ABORT_GYRO 120.0f
#define DRIVE_OVERSPEED_ABORT_ENABLE 0
#define DRIVE_ABORT_FORWARD_VELOCITY 14.0f
#define DRIVE_ABORT_CONFIRM_MS 70
#define DRIVE_ABORT_BLOCK_MS 350
#define DRIVE_SPEED_GOVERNOR_ENABLE 1
#define DRIVE_SPEED_GOVERNOR_MARGIN 0.60f
#define DRIVE_SPEED_GOVERNOR_BRAKE_RATIO 0.45f

/*------------------------Startup safety-------------------------*/
#define STARTUP_BALANCE_DELAY_MS 250
#define STARTUP_ARM_ANGLE 25.0f
#define STARTUP_ARM_GYRO 80.0f

/*------------------------Airborne / fall detect----------------*/
#define AIR_DETECT_ENABLE 0
#define FALL_PROTECT_ENABLE 1
#define FALL_PROTECT_ANGLE 45.0f
#define SUSPEND_WHEEL_VELOCITY 35.0f
#define SUSPEND_TARGET_VELOCITY 1.0f
#define SUSPEND_ACC_MIN 0.65f
#define SUSPEND_ACC_MAX 1.65f
#define SUSPEND_CONFIRM_MS 120
#define SUSPEND_REARM_DELAY_MS 150
#define LAND_READY_ANGLE 22.0f
#define LAND_READY_GYRO 70.0f
#define LAND_READY_WHEEL_VELOCITY 8.0f
#define LAND_READY_ACC_MIN 0.75f
#define LAND_READY_ACC_MAX 1.35f
#define LAND_STABLE_MS 100
#define LAND_CONTACT_REQUIRED 1
#define LAND_CONTACT_ACC_MIN 0.88f
#define LAND_CONTACT_ACC_MAX 1.18f
#define LAND_CONTACT_ACC_DELTA 0.08f

/*------------------------Battery (3S Li-ion) -------------------*/
// 3S Li-ion: full 12.6V, warn 10.8V, critical 9.9V, empty 9.0V
// Adjust BATTERY_DIVIDER_RATIO to match your voltage divider so that
// the ADC pin sees <= 3.3V at full battery.
#define BATTERY_DIVIDER_RATIO 4.0f
#define BATTERY_FULL_VOLTAGE 12.60f
#define BATTERY_EMPTY_VOLTAGE 9.00f
#define BATTERY_WARN_VOLTAGE 10.80f
#define BATTERY_CRITICAL_VOLTAGE 9.90f
#define BATTERY_CELL_COUNT 3
#define BATTERY_ADC_SAMPLES 16
#define BATTERY_FILTER_ALPHA 0.18f
#define VREF 3300

/*------------------------MPU-----------------------------------*/
#define GYRO_COEF 0.975f
#define GYRO_COEFX 0.85f
#define PRE_GYRO_COEFX 0.0f

/*------------------------Monitor Mode--------------------------*/
#define MONITOR_MODE 0

/*-------------------------------------------------------------*/
#endif
