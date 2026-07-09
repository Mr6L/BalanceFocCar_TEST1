/*
  Balance FOC Car main entry.
*/

#include <Arduino.h>
#include "APP.h"
#include "BluetoothCarControl.h"
#include "ButtonAndBattery.h"
#include "SuperCar.h"
#include "esp_system.h"

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.print("Boot reset reason: ");
    Serial.println((int)esp_reset_reason());

    Serial.println("Starting ButtonAndPower...");
    ButtonAndPower.startTask();
    delay(1000);

    Serial.println("Starting XiaoZhi BLE...");
    BluetoothCarControl.startTask();

    Serial.println("Starting APP...");
    APP.startTask();

    Serial.println("Starting BalanceCar...");
    BalanceCar.startTask();
    Serial.println("Setup completed.");
}

void loop()
{
    BalanceCar.running();
}
