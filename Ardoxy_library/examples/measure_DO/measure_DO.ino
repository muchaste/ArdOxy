/*
  Ardoxy example

  Trigger a measurement sequence (DO, temperature, air pressure) and read out the results. Print to the serial monitor.

  The circuit:
  - Arduino Uno
  - FireStingO2 connected to Pins 9,10

  created 24 October 2021
  by Stefan Mucha

*/

#include <Ardoxy.h>
#include <SoftwareSerial.h>

// Define variables
long result;                                // for measurement result
int check;                                  // numerical indicator of succesful measurement (1: success, 0: no connection, 9: mismatch)
char SeqMeasCom[7] = "SEQ1\r";              // measurement commmand that is sent to sensor during void toggleMeasurement(). Number indicates measurement channel
char DOReadCom[11] = "REA1 3 4\r";          // template for DO-read command that is sent to sensor during void toggleRead() (length = 10 because of /0 string terminator)

// Initiate connection via SoftwareSerial and create Ardoxy instance
SoftwareSerial mySer(10, 9);
Ardoxy ardoxy(&mySer);

void setup() {
  Serial.begin(19200);
  ardoxy.begin(19200);
  delay(2000);              // Some delay is necessary before the first measurement
}

void loop() {
  check = ardoxy.measure(SeqMeasCom);
  Serial.print("Measurement status: ");
  Serial.println(check);
  delay(1000);
  result = ardoxy.readout(DOReadCom);
  Serial.print("Dissolved oxygen: ");
  Serial.println(result);
  delay(1000);
}
