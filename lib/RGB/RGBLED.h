#ifndef RGBLED_h
#define RGBLED_h

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#define RL_NUMS 6
#define RL_LEFT_START 0
#define RL_LEFT_COUNT 3
#define RL_RIGHT_START 3
#define RL_RIGHT_COUNT 3
#define RL_PIN 38
#define FB_NUMS 8
#define FB_FRONT_START 0
#define FB_FRONT_COUNT 6
#define FB_REAR_START 6
#define FB_REAR_COUNT 2
#define FB_PIN 39

class RGBLED
{
  public:
    void Initialize();
    void extractRGB(const String &colorStr, uint8_t &r, uint8_t &g, uint8_t &b);
    void ResetColor(String color);
    void SetFrontColor(String color);
    void SetRearColor(String color);
    void SetFrontBackColor(String color);
    void TurnOff();
    void TurnOffFront();
    void TurnOffRear();
    void TurnOffFrontBack();
    void BootAnimation();
    void FrontStartupBlink();
};

extern RGBLED rgb;

#endif
