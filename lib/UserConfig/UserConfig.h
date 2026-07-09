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
#ifndef UserConfg_h
#define UserConfg_h

/*--------------------WiFi AP Config--------------------------*/
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

// Used only when WIFI_AP_MODE is set to 0.
#define USER_SSID ""
#define USER_PASSWORD ""

/*--------------------Remote server control--------------------*/
#define REMOTE_CONTROL_ENABLE 1
#define REMOTE_SERVER_URL ""
#define REMOTE_POLL_INTERVAL_MS 120
#define REMOTE_STATUS_INTERVAL_MS 600
#define REMOTE_HTTP_TIMEOUT_MS 700
#define WIFI_STA_CONNECT_TIMEOUT_MS 12000

/*--------------------XiaoZhi BLE control----------------------*/
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
#define XIAOZHI_VOICE_MAX_VELOCITY 0.35f
#define XIAOZHI_VOICE_MAX_STEER 1.2f
#define XIAOZHI_VOICE_THROTTLE_SIGN -1.0f
#define XIAOZHI_VOICE_START_ANGLE 5.0f
#define XIAOZHI_VOICE_START_GYRO 60.0f
#define XIAOZHI_VOICE_ABORT_ANGLE 10.0f
#define XIAOZHI_VOICE_ABORT_GYRO 130.0f

/*--------------------IP Config-------------------------------*/

#define STATIC_IP_MODE 0 // 0:DHCP 1:静态IP
#define YOUR_IP 229

/*--------------------------PID Config------------------------*/
#define VEL_Kp 0.075f // MOTOR-FOC
#define VEL_Ki 1.8f
#define C_Kp 0.6f
#define C_Ki 0.8f
#define C_LF 0.1f // MOTOR-FOC

#define VELOCITY_Kp 1.2f
#define VELOCITY_Ki 0.0f
#define VELOCITY_Kd 0.0f
#define VELOCITY_LIMIT 45.0f
#define ENABLE_VELOCITY_LOOP 1
#define VELOCITY_FEEDBACK_SIGN 1.0f
#define TARGET_ANGLE_LIMIT 12.0f
#define DIRECT_SPEED_DAMPING_ENABLE 1
#define DIRECT_SPEED_DAMPING_Kp 0.20f
#define DIRECT_SPEED_DAMPING_LIMIT 12.0f
#define DIRECT_SPEED_DAMPING_DEADBAND 0.45f
#define DIRECT_SPEED_DAMPING_FILTER_ALPHA 0.18f

#define UPRIGHT_Kp 1.05f
#define UPRIGHT_Ki 0.0f
#define UPRIGHT_Kd 0.02f
#define UPRIGHT_LIMIT 75.0f

#define TARGET_VEL_LIMIT 30.0f

#define STR_LIMIT 30.0f

#define VEL_PID_UPDATE 4
#define UPRIGHT_PID_UPDATE 2

/*------------------------WiFi drive control---------------------*/
#define WIFI_DRIVE_MAX_VELOCITY 25.0f
#define WIFI_DRIVE_MAX_STEER 20.0f
#define DRIVE_FIXED_THROTTLE_RATIO 0.30f
#define DRIVE_FIXED_TURN_RATIO 0.20f
#define WIFI_DRIVE_TIMEOUT_MS 500
#define WIFI_DRIVE_VEL_RAMP_STEP 0.02f
#define WIFI_DRIVE_BRAKE_RAMP_STEP 0.20f
#define WIFI_DRIVE_STEER_RAMP_STEP 0.35f
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

/*------------------------Distance drive-------------------------*/
#define WHEEL_DIAMETER_M 0.052f
#define WHEEL_RADIUS_M (WHEEL_DIAMETER_M * 0.5f)
#define DISTANCE_FORWARD_CALIBRATION 2.20f
#define DISTANCE_BACKWARD_CALIBRATION 1.70f
#define DISTANCE_MAX_METERS 5.0f
#define DISTANCE_MIN_SPEED_RATIO 0.12f
#define DISTANCE_DEFAULT_SPEED_RATIO 0.35f
#define DISTANCE_START_RAMP_MS 1200

/*------------------------Motor pins-----------------------------*/
#define MOTOR_A_PWM_U 35
#define MOTOR_A_PWM_V 34
#define MOTOR_A_PWM_W 33
#define MOTOR_B_PWM_U 12
#define MOTOR_B_PWM_V 11
#define MOTOR_B_PWM_W 10
#define MOTOR_VOLTAGE_LIMIT 2.0f

/*------------------------Startup safety-------------------------*/
// Set to 0 after the motor, rotor magnet, encoder, or phase wires have been
// disassembled. Stored zero angles are only valid for the exact old assembly.
#define STARTUP_SKIP_FOC_ALIGN 1
#define MOTOR_A_ZERO_ELECTRIC_ANGLE 1.446544f
#define MOTOR_B_ZERO_ELECTRIC_ANGLE 4.169360f
#define MOTOR_A_SENSOR_DIRECTION Direction::CCW
#define MOTOR_B_SENSOR_DIRECTION Direction::CCW

/*------------------------Wheel direction------------------------*/
// For the mirrored left/right wheels, forward balance torque is normally
// opposite motor shaft signs. Flip only the side that runs the wrong physical
// direction after FOC alignment is correct.
#define MOTOR_A_BALANCE_SIGN 1.0f
#define MOTOR_B_BALANCE_SIGN -1.0f
#define MOTOR_A_STEER_SIGN 1.0f
#define MOTOR_B_STEER_SIGN 1.0f
#define STARTUP_BALANCE_DELAY_MS 250
#define STARTUP_ARM_ANGLE 25.0f
#define STARTUP_ARM_GYRO 80.0f

/*------------------------Airborne detect------------------------*/
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

/*------------------------MPU-----------------------------------*/
#define GYRO_COEF 0.975f
#define GYRO_COEFX 0.85f
#define PRE_GYRO_COEFX 0.0f
/*------------------------Monitor Mode--------------------------*/

#define MONITOR_MODE 0 // 0:关闭 1:开启

/*-------------------------------------------------------------*/
#endif
