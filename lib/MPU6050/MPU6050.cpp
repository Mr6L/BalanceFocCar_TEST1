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

#include "MPU6050.h"
#include "Arduino.h"
#include "UserConfig.h"
MPU6050::MPU6050(TwoWire &w)
{
    wire = &w;

    gyroCoef = GYRO_COEF;   // 0.975f;
    gyroCoefX = GYRO_COEFX; // 0.85f;
    preGyroCoefX = PRE_GYRO_COEFX;
    accCoef = 1.0f - gyroCoef; // 0.025f;
    gyroXoffset = gyroYoffset = gyroZoffset = 0.0f;
    accX = accY = accZ = tempC = gyroX = gyroY = gyroZ = gyroXFV = 0.0f;
    angleGyroX = angleGyroY = angleGyroZ = angleAccX = angleAccY = angleAccZ = 0.0f;
    angleX = angleY = angleZ = interval = preGyroX = 0.0f;
    preInterval = 0;
}

MPU6050::MPU6050(TwoWire &w, float aC, float gC, float gCX)
{
    wire = &w;
    accCoef = aC;
    gyroCoef = gC;
    gyroCoefX = gCX;
    preGyroCoefX = PRE_GYRO_COEFX;
    gyroXoffset = gyroYoffset = gyroZoffset = 0.0f;
    accX = accY = accZ = tempC = gyroX = gyroY = gyroZ = gyroXFV = 0.0f;
    angleGyroX = angleGyroY = angleGyroZ = angleAccX = angleAccY = angleAccZ = 0.0f;
    angleX = angleY = angleZ = interval = preGyroX = 0.0f;
    preInterval = 0;
}

void MPU6050::begin()
{
    writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
    writeMPU6050(MPU6050_CONFIG, 0x00);
    writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
    writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
    writeMPU6050(MPU6050_PWR_MGMT_1, 0x00);
    this->update();
    angleGyroX = 0.0f;
    angleGyroY = 0.0f;
    angleGyroZ = 0.0f;
    gyroXFV = 0.0f;
    preGyroX = 0.0f;
    angleX = this->getAccAngleX();
    angleY = this->getAccAngleY();
    angleZ = 0.0f;
    preInterval = millis();
}

void MPU6050::writeMPU6050(byte reg, byte data)
{
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(reg);
    wire->write(data);
    wire->endTransmission();
}

byte MPU6050::readMPU6050(byte reg)
{
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(reg);
    wire->endTransmission(true);
    wire->requestFrom(MPU6050_ADDR, 1);
    byte data = wire->read();
    return data;
}

void MPU6050::setGyroOffsets(float x, float y, float z)
{
    gyroXoffset = x;
    gyroYoffset = y;
    gyroZoffset = z;
}

void MPU6050::calcGyroOffsets(bool console, uint16_t delayBefore, uint16_t delayAfter)
{
    float x = 0, y = 0, z = 0;
    int16_t rx, ry, rz;

    delay(delayBefore);
    if (console)
    {
        Serial.println();
        Serial.println("========================================");
        Serial.println("Calculating gyro offsets");
        Serial.print("DO NOT MOVE MPU6050");
    }
    for (int i = 0; i < 3000; i++)
    {
        if (console && i % 1000 == 0)
        {
            Serial.print(".");
        }
        wire->beginTransmission(MPU6050_ADDR);
        wire->write(0x43);
        wire->endTransmission(false);
        wire->requestFrom((int)MPU6050_ADDR, 6);

        rx = wire->read() << 8 | wire->read();
        ry = wire->read() << 8 | wire->read();
        rz = wire->read() << 8 | wire->read();

        x += ((float)rx) / 65.5;
        y += ((float)ry) / 65.5;
        z += ((float)rz) / 65.5;
    }
    gyroXoffset = x / 3000;
    gyroYoffset = y / 3000;
    gyroZoffset = z / 3000;

    if (console)
    {
        Serial.println();
        Serial.println("Done!");
        Serial.print("X : ");
        Serial.println(gyroXoffset);
        Serial.print("Y : ");
        Serial.println(gyroYoffset);
        Serial.print("Z : ");
        Serial.println(gyroZoffset);
        Serial.println("Program will start after 3 seconds");
        Serial.print("========================================");
        delay(delayAfter);
    }
}

void MPU6050::update()
{
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(0x3B);
    wire->endTransmission(false);
    wire->requestFrom((int)MPU6050_ADDR, 14);

    rawAccX = wire->read() << 8 | wire->read();
    rawAccY = wire->read() << 8 | wire->read();
    rawAccZ = wire->read() << 8 | wire->read();
    rawTemp = wire->read() << 8 | wire->read();
    rawGyroX = wire->read() << 8 | wire->read();
    rawGyroY = wire->read() << 8 | wire->read();
    rawGyroZ = wire->read() << 8 | wire->read();

    accX = ((float)rawAccX) / 16384.0;
    accY = ((float)rawAccY) / 16384.0;
    accZ = ((float)rawAccZ) / 16384.0;
    tempC = ((float)rawTemp) / 340.0f + 36.53f;

    angleAccX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;
    angleAccY = atan2(-accX, accZ + abs(accY)) * 360 / 2.0 / PI;

    gyroX = ((float)rawGyroX) / 65.5;
    gyroY = ((float)rawGyroY) / 65.5;
    gyroZ = ((float)rawGyroZ) / 65.5;

    gyroX -= gyroXoffset;
    gyroY -= gyroYoffset;
    gyroZ -= gyroZoffset;

    interval = (millis() - preInterval) * 0.001;

    angleGyroX += gyroX * interval;
    angleGyroY += gyroY * interval;
    angleGyroZ += gyroZ * interval;

    angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
    angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
    angleZ = angleGyroZ;

    preInterval = millis();
}
