/*
  Ardoxy.h - Library for communicating with a FireSting Oxygen sensor.
  Created by Stefan Mucha, October 23, 2021.
*/

#ifndef Ardoxy_h
#define Ardoxy_h

#include "Arduino.h"
#include <SoftwareSerial.h>

class Ardoxy
{
  public:
    Ardoxy(SoftwareSerial * ss);
    void begin(uint32_t baud);
    void end();
    int measure(char command[]);
    long readout(char command[]);
  private:
    SoftwareSerial * FS_serial;
};

#endif
