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

#ifndef MPU6050_H
#define MPU6050_H

#include "Arduino.h"
#include "Wire.h"

#define MPU6050_ADDR 0x68
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_TEMP_H 0x41
#define MPU6050_TEMP_L 0x42

class MPU6050
{
  public:
    MPU6050(TwoWire &w);
    MPU6050(TwoWire &w, float aC, float gC, float gCX);

    void begin();

    void setGyroOffsets(float x, float y, float z);

    void writeMPU6050(byte reg, byte data);
    byte readMPU6050(byte reg);

    int16_t getRawAccX()
    {
        return rawAccX;
    };
    int16_t getRawAccY()
    {
        return rawAccY;
    };
    int16_t getRawAccZ()
    {
        return rawAccZ;
    };

    int16_t getRawTemp()
    {
        return rawTemp;
    };
    float getTempC()
    {
        return tempC;
    };

    int16_t getRawGyroX()
    {
        return rawGyroX;
    };
    int16_t getRawGyroY()
    {
        return rawGyroY;
    };
    int16_t getRawGyroZ()
    {
        return rawGyroZ;
    };

    float getGyroXFV()
    {
        gyroXFV = gyroCoefX * gyroX + (1 - gyroCoefX) * gyroXFV + preGyroCoefX * preGyroX;
        // gyroXFV = gyroCoefX * gyroX + (1 - gyroCoefX) * preGyroX;
        preGyroX = gyroX;
        return gyroXFV;
    };

    float getAccX()
    {
        return accX;
    };
    float getAccY()
    {
        return accY;
    };
    float getAccZ()
    {
        return accZ;
    };

    float getGyroX()
    {
        return gyroX;
    };
    float getGyroY()
    {
        return gyroY;
    };
    float getGyroZ()
    {
        return gyroZ;
    };

    void calcGyroOffsets(bool console = false, uint16_t delayBefore = 1000, uint16_t delayAfter = 800);

    float getGyroXoffset()
    {
        return gyroXoffset;
    };
    float getGyroYoffset()
    {
        return gyroYoffset;
    };
    float getGyroZoffset()
    {
        return gyroZoffset;
    };

    void update();

    float getAccAngleX()
    {
        return angleAccX;
    };
    float getAccAngleY()
    {
        return angleAccY;
    };

    float getGyroAngleX()
    {
        return angleGyroX;
    };
    float getGyroAngleY()
    {
        return angleGyroY;
    };
    float getGyroAngleZ()
    {
        return angleGyroZ;
    };

    float getAngleX()
    {
        return angleX;
    };
    float getAngleY()
    {
        return angleY;
    };
    float getAngleZ()
    {
        return angleZ;
    };

  private:
    TwoWire *wire;

    int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;

    float gyroXoffset, gyroYoffset, gyroZoffset;

    float accX, accY, accZ, tempC, gyroX, gyroY, gyroZ, gyroXFV;

    float angleGyroX, angleGyroY, angleGyroZ, angleAccX, angleAccY, angleAccZ;

    float angleX, angleY, angleZ;

    float interval, preGyroX;
    long preInterval;

    float accCoef, gyroCoef, gyroCoefX, preGyroCoefX;
};

#endif
