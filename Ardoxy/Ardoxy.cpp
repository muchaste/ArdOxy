/*
  Ardoxy.h - Library for communicating with a FireSting Oxygen sensor.
  Created by Stefan Mucha, October 23, 2021.
*/

#include <Arduino.h>
#include <Ardoxy.h>
#include <SoftwareSerial.h>


void Ardoxy::begin()
{

  if (hwStream)
  {
    hwStream->begin(19200);
    delay(200);
    while(hwStream->available()>0){
      char t = hwStream->read();
      delay(2);
    }
    hwStream->write("#VERS\r");
    delay(200);
    if(hwStream->available()){
      Serial.println("Connection established, Firmware 3.xx");
      ver = 3;
      while(hwStream->available()>0){
        char t = hwStream->read();
        delay(2);
      }
    } else {
      hwStream->begin(115200);
      delay(200);
      while(hwStream->available()>0){
        char t = hwStream->read();
        delay(2);
      }
      hwStream->write("#VERS\r");
      delay(200);
      if(hwStream->available()){
        Serial.println("Connection established, Firmware 4.xx");
        ver = 4;
        while(hwStream->available()>0){
          char t = hwStream->read();
          delay(2);
        }
      } else {
        Serial.println("Couldn't establish connection");
        ver = 0;
      }
    }
  }
  else
  {
    swStream->begin(19200);
    delay(200);
    while(swStream->available()>0){
      char t = swStream->read();
      delay(2);
    }
    swStream->write("#VERS\r");
    delay(200);
    if(swStream->available()){
      Serial.println("Connection established, Firmware 3.xx");
      ver = 3;
      while(swStream->available()>0){
        char t = swStream->read();
        delay(2);
      }
    } else {
      swStream->begin(115200);
      delay(200);
      while(swStream->available()>0){
        char t = swStream->read();
        delay(2);
      }
      swStream->write("#VERS\r");
      delay(200);
      if(swStream->available()){
        Serial.println("Connection established, Firmware 4.xx");
        ver = 4;
        while(swStream->available()>0){
          char t = swStream->read();
          delay(2);
        }
      } else {
        Serial.println("Couldn't establish connection");
        ver = 0;
      }
    }
  }
}


// End function
void Ardoxy::end()
{
  if (hwStream)
  {
    hwStream->end();
  }
  else
  {
    swStream->end();
  }
}

// Measure function: send measurement command to firesting via Serial communication
// Returns:
// 1 when echo matches command
// 0 when there is no echo (connection problem)
// 9 when there is a mismatch (usually due to timing or connection issues)
int Ardoxy::measure(char command[])
{
  int result;

  // Set source Stream
  stream = !hwStream? (Stream*)swStream : hwStream;

  // Empty Serial buffer
  while(stream->available() > 0){
    char t = stream->read();
    delay(2);
  }

  // Send command to FireSting
  stream->write(command);
  stream->flush();
  bool received = false;      // Switch to continue reading incoming data until end marker was received
  delay(500);                 // Let Firesting finish measurement before reading incoming serial data

  if(!stream->available()){ // If there is no incoming data, there is a connection problem
    result = 0;
  }
  else{
    while (stream->available() > 0 && received == false && ndx <= numChars-1) {
      delay(2);
      rc = stream->read();
      if (rc != endMarker && ndx < numChars-1) {
        receivedChars[ndx] = rc;
        ndx++;
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

// Measure Sequence function: same as measure function but with pre-set measurement command
int Ardoxy::measureSeq(int chan)
{
  int result;

  // Paste Channel in measurement command
  if(ver == 4){
    sprintf(seqCommand, "MEA %d 47\r", chan);           // insert channel in measurement command
  } else if (ver == 3) {
    sprintf(seqCommand, "SEQ %d\r", chan);           // insert channel in measurement command
  }

  // Set source Stream
  stream = !hwStream? (Stream*)swStream : hwStream;

  // Empty Serial buffer
  while(stream->available() > 0){
    char t = stream->read();
    delay(2);
  }

  // Send command to FireSting
  stream->write(seqCommand);
  stream->flush();
  bool received = false;      // Switch to continue reading incoming data until end marker was received
  delay(500);                 // Let Firesting finish measurement before reading incoming serial data

  if(!stream->available()){ // If there is no incoming data, there is a connection problem
    result = 0;
  }
  else{
    while (stream->available() > 0 && received == false && ndx <= numChars-1) {
      delay(2);
      rc = stream->read();
      if (rc != endMarker && ndx < numChars-1) {
        receivedChars[ndx] = rc;
        ndx++;
      }
      else {
        receivedChars[ndx] = '\0';  // terminate the string
        ndx = 0;
        received = true;
        if(strncmp(seqCommand, receivedChars, (strlen(seqCommand)-1)) == 0){ // Compare command and received string (FS echoes the command)
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
  long valInt;                                                                          // receives parsed numerical value

  // Set source Stream
  stream = !hwStream? (Stream*)swStream : hwStream;

  // Empty Serial buffer
  while(stream->available() > 0){
    char t = stream->read();
    delay(2);
  }

  stream->write(command);
  stream->flush();
  bool received = false;
  delay(100);
  while (stream->available() > 0 && received == false && ndx <= numChars-1) {          // only read serial data if the buffer was emptied before and it's new data
    delay(2);
    rc = stream->read();
    if (rc != endMarker && ndx < numChars-1) {
      receivedChars[ndx] = rc;                                                        // store the latest character in character array
      ndx++;
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
