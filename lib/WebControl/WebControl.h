#ifndef WEB_CONTROL_H
#define WEB_CONTROL_H

#include <Arduino.h>

class WebControlTask
{
  public:
    void startTask();
};

extern WebControlTask WebControl;

#endif
