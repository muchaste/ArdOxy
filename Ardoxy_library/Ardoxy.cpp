/*
  Ardoxy.h - Library for communicating with a FireSting Oxygen sensor.
  Created by Stefan Mucha, October 23, 2021.
*/

#include <Arduino.h>
#include <Ardoxy.h>
#include <SoftwareSerial.h>

Ardoxy::Ardoxy(SoftwareSerial * ss)
{
  FS_serial = ss;
}

void Ardoxy::begin(uint32_t baud)
{
  FS_serial->begin(baud);
}

// Measure function: send measurement command to firesting via Serial communication
// Returns:
// 1 when echo matches command
// 0 when there is no echo (connection problem)
// 9 when there is a mismatch (usually due to timing or connection issues)
int Ardoxy::measure(char command[])
{
  int numChars = 30;                                                                    // length of preallocated char array
  static byte ndx = 0;                                                                  // index for storing in the array
  char receivedChars[numChars];                                                         // Array to hold incoming data
  char endMarker = '\r';                                                                // declare the character that marks the end of a serial transmission
  char rc;                                                                              // temporary variable to hold the last received character
  int result;

  // Send command to FireSting
  FS_serial->write(command);
  FS_serial->flush();
  bool received = false;      // Switch to continue reading incoming data until end marker was received
  delay(700);                 // Let Firesting finish measurement before reading incoming serial data

  if(!FS_serial->available()){ // If there is no incoming data, there is a connection problem
    result = 0;
  }
  else{
    while (FS_serial->available() > 0 && received == false) {
      delay(2);
      rc = FS_serial->read();
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) { // If more than the preallocated number of characters is received, insert at end of the char array
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0';  // terminate the string
        ndx = 0;
        received = true;
        if(strncmp(command, receivedChars, (strlen(command)-1)) == 0){ // Compare command and received string (FS echoes the command)
          result = 1;
        }
        else{
          result = 9;             // return 9 if there is a mismatch
        }
      }
    }
  }
  return result;
}

// Readout values from Firesting memory
// Returns:
// numerical value (air saturation or temperature) - refer to Firesting Protocol
// 0 if there is a communication mismatch
long Ardoxy::readout(char command[])
{                                       // receives serial data and stores it in array until endmarker is received
  int numChars = 30;                                                                    // length of preallocated char array
  long valInt;                                                                          // receives parsed numerical value
  static byte ndx = 0;                                                                  // index for storing in the array
  char receivedChars[numChars];                                                         // Array to hold incoming data
  char endMarker = '\r';                                                                // declare the character that marks the end of a serial transmission
  char rc;                                                                              // temporary variable to hold the last received character

  FS_serial->write(command);
  FS_serial->flush();
  bool received = false;
  delay(100);
  while (FS_serial->available() > 0 && received == false) {          // only read serial data if the buffer was emptied before and it's new data
    delay(2);
    rc = FS_serial->read();
    if (rc != endMarker) {
      receivedChars[ndx] = rc;                                                        // store the latest character in character array
      ndx++;
      if (ndx >= numChars) {                                                          // make sure that array size is not exceeded
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0';                                                      // terminate the string if the end marker is received
      ndx = 0;
      if(strncmp(command, receivedChars, (strlen(command)-1)) == 0){
        received = true;
        char* separator = strrchr(receivedChars, ' ');    // the value is separated by a space -> strrchr finds the last occurrence of " " in receivedChars and points
                                                          // to the following part of the string
        valInt = atol(separator);                         // parse the character to integers - the air saturation values are returned as [% air saturation x 1000], temperature as [°C x 1000]
      }
      else{
        valInt = 0;
      }
    }
  }
  return valInt;
}
