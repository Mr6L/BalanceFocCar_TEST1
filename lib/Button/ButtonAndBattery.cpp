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

#include "ButtonAndBattery.h"
#include "BuzzerSound.h"
#include "OneButton.h"
#include "RGBLED.h"
#include "UserConfig.h"
#include <math.h>

OneButton bnt(BUTTON_PIN, true);
volatile uint16_t batteryAdcRaw = 0;
volatile uint32_t batteryAdcMilliVolts = 0;
volatile float batteryVoltage = 0.0f;
volatile uint8_t batteryPercent = 0;
volatile bool batteryLow = false;
volatile bool batteryCritical = false;
static unsigned long lastLowBatteryAlarmMs = 0;
static float filteredBatteryVoltage = 0.0f;

static void prepareMotorPwmPins()
{
    pinMode(MOTOR_A_PWM_U, OUTPUT);
    pinMode(MOTOR_A_PWM_V, OUTPUT);
    pinMode(MOTOR_A_PWM_W, OUTPUT);
    pinMode(MOTOR_B_PWM_U, OUTPUT);
    pinMode(MOTOR_B_PWM_V, OUTPUT);
    pinMode(MOTOR_B_PWM_W, OUTPUT);
    digitalWrite(MOTOR_A_PWM_U, LOW);
    digitalWrite(MOTOR_A_PWM_V, LOW);
    digitalWrite(MOTOR_A_PWM_W, LOW);
    digitalWrite(MOTOR_B_PWM_U, LOW);
    digitalWrite(MOTOR_B_PWM_V, LOW);
    digitalWrite(MOTOR_B_PWM_W, LOW);
}

void balanceCarPowerOn()
{
    digitalWrite(POWEREN_PIN, HIGH);
}

void balanceCarPowerOff()
{
    buzzer.play(powerOffID);
    rgb.TurnOff();
    digitalWrite(POWEREN_PIN, LOW);
}

void turnOffRGB()
{
    buzzer.play(powerOffledID);
    rgb.TurnOff();
}

void focSelfCheckCompleted()
{
    buzzer.play(S_BEEP);
}

void startPostInitIndicators()
{
    buzzer.play(powerOnID);
    rgb.Initialize();
    focSelfCheckCompleted();
}

static uint8_t batteryPercentFromVoltage(float voltage)
{
    if (voltage <= BATTERY_EMPTY_VOLTAGE)
    {
        return 0;
    }
    if (voltage >= BATTERY_FULL_VOLTAGE)
    {
        return 100;
    }

    return (uint8_t)constrain(
        lroundf((voltage - BATTERY_EMPTY_VOLTAGE) * 100.0f / (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE)),
        0L,
        100L);
}

static const char *batteryLevelName()
{
    if (batteryCritical)
    {
        return "critical";
    }
    if (batteryLow)
    {
        return "low";
    }
    return "normal";
}

String batteryStatusJson()
{
    String json = "{";
    json += "\"voltage\":" + String(batteryVoltage, 3);
    json += ",\"percent\":" + String(batteryPercent);
    json += ",\"cell_voltage\":" + String(batteryVoltage / 2.0f, 3);
    json += ",\"low\":" + String(batteryLow ? 1 : 0);
    json += ",\"critical\":" + String(batteryCritical ? 1 : 0);
    json += ",\"level\":\"" + String(batteryLevelName()) + "\"";
    json += ",\"adc_raw\":" + String(batteryAdcRaw);
    json += ",\"adc_mv\":" + String(batteryAdcMilliVolts);
    json += ",\"divider_ratio\":" + String(BATTERY_DIVIDER_RATIO, 3);
    json += ",\"full_voltage\":" + String(BATTERY_FULL_VOLTAGE, 2);
    json += ",\"empty_voltage\":" + String(BATTERY_EMPTY_VOLTAGE, 2);
    json += "}";
    return json;
}

void CheckBatteryVoltageForSafety()
{
    uint32_t rawSum = 0;
    uint32_t mvSum = 0;

    for (uint8_t i = 0; i < BATTERY_ADC_SAMPLES; ++i)
    {
        rawSum += analogRead(BAT_ADC_PIN);
        mvSum += analogReadMilliVolts(BAT_ADC_PIN);
        delayMicroseconds(150);
    }

    batteryAdcRaw = rawSum / BATTERY_ADC_SAMPLES;
    batteryAdcMilliVolts = mvSum / BATTERY_ADC_SAMPLES;

    float adcVoltage = batteryAdcMilliVolts > 0
                           ? (float)batteryAdcMilliVolts / 1000.0f
                           : ((float)batteryAdcRaw * (float)VREF / 1000.0f / 4095.0f);
    const float measuredBatteryVoltage = adcVoltage * BATTERY_DIVIDER_RATIO;

    if (filteredBatteryVoltage < 0.1f)
    {
        filteredBatteryVoltage = measuredBatteryVoltage;
    }
    else
    {
        filteredBatteryVoltage =
            filteredBatteryVoltage * (1.0f - BATTERY_FILTER_ALPHA) + measuredBatteryVoltage * BATTERY_FILTER_ALPHA;
    }

    batteryVoltage = filteredBatteryVoltage;
    batteryPercent = batteryPercentFromVoltage(filteredBatteryVoltage);
    batteryLow = filteredBatteryVoltage <= BATTERY_WARN_VOLTAGE;
    batteryCritical = filteredBatteryVoltage <= BATTERY_CRITICAL_VOLTAGE;

    if (batteryLow && millis() - lastLowBatteryAlarmMs >= 5000)
    {
        lastLowBatteryAlarmMs = millis();
        Serial.print("Low battery: ");
        Serial.print(batteryVoltage, 3);
        Serial.print("V (");
        Serial.print(batteryPercent);
        Serial.print("%), ADC=");
        Serial.print(batteryAdcRaw);
        Serial.print(", mv=");
        Serial.println(batteryAdcMilliVolts);
        buzzer.play(S_SIREN);
    }
}

void ButtonEventTask(void *pvParameters)
{

    pinMode(POWEREN_PIN, OUTPUT);
    balanceCarPowerOn();

    buzzer.Init(BUZZER_PIN);

    bnt.reset();
    bnt.attachLongPressStart(balanceCarPowerOff);
    bnt.attachClick(turnOffRGB);
    // bnt.attachDoubleClick(turnOffRGB);

    for (;;)
    {
        bnt.tick();
        vTaskDelay(50);
    }
};

void BatteryVoltageCheckTask(void *pvParameters)
{
    pinMode(BAT_ADC_PIN, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
    for (;;)
    {
        CheckBatteryVoltageForSafety();
        vTaskDelay(500);
    }
};

void BNT_AND_PWR::startTask()
{
    pinMode(POWEREN_PIN, OUTPUT);
    prepareMotorPwmPins();
    balanceCarPowerOn();

    xTaskCreatePinnedToCore(ButtonEventTask, "Button_Event", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(BatteryVoltageCheckTask, "Battery_Voltage_Check", 2048, NULL, 5, NULL, 1);
};

BNT_AND_PWR ButtonAndPower;
