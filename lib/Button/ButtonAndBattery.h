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
#ifndef ButtonAndBattery_h
#define ButtonAndBattery_h
#include <Arduino.h>
// ----------------------
#define BUTTON_PIN 6  // 按键引脚
#define BUZZER_PIN 40 // 蜂鸣器引脚
#define POWEREN_PIN 7 // 电源引脚
#define BAT_ADC_PIN 1
#define BATTERY_DIVIDER_RATIO 3.0f
#define BATTERY_FULL_VOLTAGE 8.40f
#define BATTERY_EMPTY_VOLTAGE 6.40f
#define BATTERY_WARN_VOLTAGE 6.80f
#define BATTERY_CRITICAL_VOLTAGE 6.40f
#define BATTERY_ADC_SAMPLES 16
#define BATTERY_FILTER_ALPHA 0.18f
#define VREF 3280 // 参考电压3.3V,供电8.15V分压 2.7V:(2.7/3.3)*4095 =  3350
//--------------------- SOUND ID ---------------------
#define changeColorID 11
#define powerOnID 4
#define powerOffID 5
#define powerOffledID 1
// ---------------------- button class -------------------------
class BNT_AND_PWR
{
  public:
    void startTask();
};
void balanceCarPowerOff();
void focSelfCheckCompleted();
void startPostInitIndicators();
void turnOffRGB();

extern BNT_AND_PWR ButtonAndPower;
extern volatile uint16_t batteryAdcRaw;
extern volatile uint32_t batteryAdcMilliVolts;
extern volatile float batteryVoltage;
extern volatile uint8_t batteryPercent;
extern volatile bool batteryLow;
extern volatile bool batteryCritical;

String batteryStatusJson();

#endif
