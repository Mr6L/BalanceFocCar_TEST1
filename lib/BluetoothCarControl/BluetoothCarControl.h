#ifndef BluetoothCarControl_h
#define BluetoothCarControl_h

#include <Arduino.h>

class BluetoothCarControlTask
{
  public:
    void startTask();
};

extern BluetoothCarControlTask BluetoothCarControl;

#endif
