/*
  Ardoxy example

  Trigger a measurement sequence (DO, temperature, air pressure) and read out the results. Print to the serial monitor.
  Oxygen sensor is calbrated using the Pyro Oxygen Logger Software.
  Oxygen probe is connected to channel 1.

  The circuit:
  - Arduino Uno
  - FireStingO2 - 7 pin connector:
    *Pin 1 connected to Arduino GND
    *Pin 2 connected to Arduino 5V 
    *Pin 4 connected to Arduino RX (here: 10)
    *Pin 5 connected to Arduino TX (here: 9)

  created 24 October 2021
  last revised 3 March 2022
  by Stefan Mucha

*/

#include <Ardoxy.h>
#include <SoftwareSerial.h>

// Set sampling interval in ms (due to the duration of the measurement and communication, use interval > 1000 msec)
unsigned long sampInterval = 5000;

// Define variables
long result;                                // for measurement result
double resultFloat;                         // result as floating point number
int check;                                  // numerical indicator of succesful measurement (1: success, 0: no connection, 9: mismatch)
char DOReadCom[11] = "REA 1 3 4\r";         // template for DO-read command that is sent to sensor
                                            // (numbers: 1 = channel, 3 = measurement results register, 4 = DO as % air sat*1000)
unsigned long loopStart, elapsed;           // ms timestamp of beginning and end of measurement loop
bool startTrigger = false;                  // trigger for start of measurement


// Initiate connection via SoftwareSerial and create Ardoxy instance
SoftwareSerial mySer(10, 9);
Ardoxy ardoxy(mySer);

void setup() {
  Serial.begin(19200);
  delay(300);
  Serial.println("---------------- Ardoxy measurement example ----------------");
  ardoxy.begin();
  Serial.println("FireSting channel: 1");
  Serial.print("Measurement interval (ms): ");
  Serial.println(sampInterval);
  Serial.println("Send \"1\" to start measurement and \"0\" to end measurement.");
  Serial.println("------------------------------------------------------------");

}

void loop() {
  // wait for serial input to start measurement.
  if (Serial.available() > 0) {
    switch(Serial.read()){
      case '1':
          startTrigger = true;
          Serial.println("started");
          break;
      case '0':
          startTrigger = false;
          Serial.print("stopped");
          break;
    }
  }
  
  if (startTrigger){
    loopStart = millis();                     // get time at beginning of loop
    check = ardoxy.measureSeq(1);
    Serial.print("Measurement status: ");
    Serial.println(check);
    delay(20);
    result = ardoxy.readout(DOReadCom);
    resultFloat = result/1000.00;
    Serial.print("Dissolved oxygen: ");
    Serial.println(resultFloat);
    elapsed = millis()-loopStart;
    delay(sampInterval - elapsed);
  }
}
