#ifndef CAR_COMMANDS_H
#define CAR_COMMANDS_H

#include <Arduino.h>

// Shared command structure used by BLE and WebControl to feed setpoints
// into the balance controller.
struct DriveCommand
{
    volatile float velocity = 0.0f;      // forward velocity setpoint
    volatile float steer = 0.0f;         // steering yaw-rate setpoint
    volatile bool enabled = false;       // true while a drive command is active
    volatile unsigned long timeoutMs = 0;// absolute millis() when command expires
};

extern DriveCommand driveCommand;

#endif
