/*
  Ardoxy.h - Library for communicating with a FireSting Oxygen sensor.
  Created by Stefan Mucha, October 23, 2021.
*/

#ifndef Ardoxy_h
#define Ardoxy_h

#include "Arduino.h"
#include <SoftwareSerial.h>

#define numChars 60

class Ardoxy
{
  public:
    Ardoxy( HardwareSerial& device) {hwStream = &device;}
    Ardoxy( SoftwareSerial& device) {swStream = &device;}
    void begin();
    void end();
    int getVer();
    int measure(char command[], int serialDelay=300);
    int measureSeq(int chan, int serialDelay=500);
    int measureDO(int chan, int serialDelay=100);
    int measureTemp(int serialDelay=300);
    long readout(char command[]);
    long readoutDO(int chan);
    long readoutTemp();
    static int calcDays(int startDay, int startMonth, int startYear, int endDay, int endMonth, int endYear);

  private:
    HardwareSerial* hwStream;
    SoftwareSerial* swStream;
    Stream* stream;
    int ver;
    int ndx = 0;                                                            // index for storing in the array
    char receivedChars[numChars];                                           // Array to hold incoming data
    char endMarker = '\r';                                                  // declare the character that marks the end of a serial transmission
    char rc;                                                                // temporary variable to hold the last received character
    char measCommand[13];                                                   // Buffer for measurement command
};

#endif
