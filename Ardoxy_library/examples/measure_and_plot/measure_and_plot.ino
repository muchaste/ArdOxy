/*
  Ardoxy example - measure and plot using SerialPlot (https://hackaday.io/project/5334-serialplot-realtime-plotting-software)

  Trigger a measurement sequence (DO, temperature, air pressure) and read out the results. Receive and plot with SerialPlot software.
  Oxygen sensor is calbrated using the Pyro Oxygen Logger Software.
  Oxygen probe is connected to channel 1.
  
  The circuit:
  - Arduino Uno
  - FireStingO2 - 7 pin connector:
    *Pin 1 connected to Arduino GND
    *Pin 2 connected to Arduino 5V 
    *Pin 4 connected to Arduino RX (here: 10)
    *Pin 5 connected to Arduino TX (here: 9)

  The software:
  Download SerialPlot and use the configuration file (*.ini) from the Ardoxy github repository.
  Import the settings in SerialPlot using File>>Load Settings

  created 11 November 2021
  by Stefan Mucha

*/

#include <Ardoxy.h>
#include <SoftwareSerial.h>

// Set sampling interval in ms (due to the duration of the measurement and communication, use interval > 1000 msec)
unsigned long sampInterval = 5000;

// Define variables
long DOInt, tempInt;                        // for measurement result
double DOFloat, tempFloat;                  // measurement result as floating point number
int check;                                  // numerical indicator of succesful measurement (1: success, 0: no connection, 9: mismatch)
char SeqMeasCom[7] = "SEQ 1\r";             // measurement commmand that is sent to sensor. Number indicates measurement channel
char DOReadCom[11] = "REA 1 3 4\r";         // template for DO-read command that is sent to sensor
char tempReadCom[11] = "REA 1 3 5\r";       // template for temp-read command that is sent to sensor
bool startTrigger = false;                  // trigger for start of measurement
unsigned long loopStart, elapsed;           // ms timestamp of beginning and end of measurement loop


// Initiate connection via SoftwareSerial and create Ardoxy instance
SoftwareSerial mySer(10, 9);
Ardoxy ardoxy(&mySer);

void setup() {
  Serial.begin(19200);
  ardoxy.begin(19200);
  delay(1000);              // Some delay is necessary before the first measurement
  Serial.print("Measurement interval (ms): ");
  Serial.println(sampInterval);
  Serial.println("Send \"1\" to start measurement and \"0\" to end measurement");
}

void loop() {
  // wait for serial input to start measurement.
  if (Serial.available() > 0) {
    switch(Serial.read()){
      case '1':
          startTrigger = true;
          Serial.println("DO_air_sat;Temp_deg_C");
          break;
      case '0':
          startTrigger = false;
          break;
    }
  }

  if (startTrigger) {
    loopStart = millis();                     // get time at beginning of loop
    
    // measure sequence (ONLY FIRMWARE 3.XX on FireSting)
    check = ardoxy.measure(SeqMeasCom);
    if(check == 1){
      DOInt = ardoxy.readout(DOReadCom);      // read DO value from results register
      delay(20);
      tempInt = ardoxy.readout(tempReadCom);  // read temperature value from results register
      delay(20);
      DOFloat = DOInt / 1000.00;              // convert to floating point number
      tempFloat = tempInt / 1000.00;          // convert to floating point number
      Serial.print(DOFloat);                  // print to serial
      Serial.print(";");
      Serial.println(tempFloat);
      elapsed = millis()-loopStart;
      delay(sampInterval - elapsed);          // wait for next loop iteration
    }
  }
}
