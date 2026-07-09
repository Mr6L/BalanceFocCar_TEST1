#include "RGBLED.h"

Adafruit_NeoPixel LR_Strip = Adafruit_NeoPixel(RL_NUMS, RL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel FB_Strip = Adafruit_NeoPixel(FB_NUMS, FB_PIN, NEO_GRB + NEO_KHZ800);

static void setFrontBackRange(uint8_t start, uint8_t count, uint8_t r, uint8_t g, uint8_t b)
{
    for (uint8_t offset = 0; offset < count && start + offset < FB_NUMS; offset++)
    {
        FB_Strip.setPixelColor(start + offset, r, g, b);
    }
}

void RGBLED::Initialize()
{
    LR_Strip.begin();
    FB_Strip.begin();
    LR_Strip.setBrightness(50);
    FB_Strip.setBrightness(80);
    TurnOff();
    FrontStartupBlink();
    BootAnimation();
}

void RGBLED::extractRGB(const String &colorStr, uint8_t &r, uint8_t &g, uint8_t &b)
{
    int startIndex = colorStr.indexOf('{') + 1;
    int endIndex = colorStr.lastIndexOf('}');

    if (startIndex <= 0 || endIndex <= startIndex)
    {
        r = 0;
        g = 0;
        b = 0;
        return;
    }

    String colorData = colorStr.substring(startIndex, endIndex);
    int commaIndex1 = colorData.indexOf(',');
    int commaIndex2 = colorData.indexOf(',', commaIndex1 + 1);

    if (commaIndex1 < 0 || commaIndex2 < 0)
    {
        r = 0;
        g = 0;
        b = 0;
        return;
    }

    r = constrain(colorData.substring(0, commaIndex1).toInt(), 0, 255);
    g = constrain(colorData.substring(commaIndex1 + 1, commaIndex2).toInt(), 0, 255);
    b = constrain(colorData.substring(commaIndex2 + 1).toInt(), 0, 255);
}

void RGBLED::ResetColor(String color)
{
    uint8_t r, g, b;
    extractRGB(color, r, g, b);

    for (uint8_t i = 0; i < RL_NUMS; i++)
    {
        LR_Strip.setPixelColor(i, r, g, b);
    }
    LR_Strip.show();
}

void RGBLED::SetFrontColor(String color)
{
    uint8_t r, g, b;
    extractRGB(color, r, g, b);
    setFrontBackRange(FB_FRONT_START, FB_FRONT_COUNT, r, g, b);
    FB_Strip.show();
}

void RGBLED::SetRearColor(String color)
{
    uint8_t r, g, b;
    extractRGB(color, r, g, b);
    setFrontBackRange(FB_REAR_START, FB_REAR_COUNT, r, g, b);
    FB_Strip.show();
}

void RGBLED::SetFrontBackColor(String color)
{
    uint8_t r, g, b;
    extractRGB(color, r, g, b);
    setFrontBackRange(0, FB_NUMS, r, g, b);
    FB_Strip.show();
}

void RGBLED::TurnOff()
{
    for (uint8_t i = 0; i < RL_NUMS; i++)
    {
        LR_Strip.setPixelColor(i, 0, 0, 0);
    }
    for (uint8_t i = 0; i < FB_NUMS; i++)
    {
        FB_Strip.setPixelColor(i, 0, 0, 0);
    }
    LR_Strip.show();
    FB_Strip.show();
}

void RGBLED::TurnOffFront()
{
    setFrontBackRange(FB_FRONT_START, FB_FRONT_COUNT, 0, 0, 0);
    FB_Strip.show();
}

void RGBLED::TurnOffRear()
{
    setFrontBackRange(FB_REAR_START, FB_REAR_COUNT, 0, 0, 0);
    FB_Strip.show();
}

void RGBLED::TurnOffFrontBack()
{
    setFrontBackRange(0, FB_NUMS, 0, 0, 0);
    FB_Strip.show();
}

void RGBLED::FrontStartupBlink()
{
    for (uint8_t blink = 0; blink < 2; blink++)
    {
        setFrontBackRange(FB_FRONT_START, FB_FRONT_COUNT, 120, 120, 120);
        setFrontBackRange(FB_REAR_START, FB_REAR_COUNT, 0, 0, 0);
        FB_Strip.show();
        delay(180);

        TurnOffFrontBack();
        delay(160);
    }
}

void RGBLED::BootAnimation()
{
    const uint32_t bootColor = LR_Strip.Color(0, 40, 160);

    TurnOff();
    for (uint8_t i = RL_LEFT_START; i < RL_LEFT_START + RL_LEFT_COUNT && i < RL_NUMS; i++)
    {
        LR_Strip.setPixelColor(i, bootColor);
    }
    LR_Strip.show();
    delay(160);

    TurnOff();
    for (uint8_t i = RL_RIGHT_START; i < RL_RIGHT_START + RL_RIGHT_COUNT && i < RL_NUMS; i++)
    {
        LR_Strip.setPixelColor(i, bootColor);
    }
    LR_Strip.show();
    delay(160);

    for (uint8_t i = 0; i < RL_NUMS; i++)
    {
        LR_Strip.setPixelColor(i, bootColor);
    }
    LR_Strip.show();
    delay(220);
    TurnOff();
}

RGBLED rgb;
