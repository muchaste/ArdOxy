/*
  Ardoxy example - measure, control solenoid and plot using SerialPlot (https://hackaday.io/project/5334-serialplot-realtime-plotting-software)

  Trigger a measurement sequence (DO, temperature, air pressure) and read out the results. Receive and plot with SerialPlot software.
  Set desired DO level by opening a solenoid valve. Opening time is calculated with PID library.
  
  Oxygen sensor is calbrated using the Pyro Oxygen Logger Software.
  Oxygen probe is connected to channel 1.
  
  The circuit:
  - Arduino Uno
  - FireStingO2 - 7 pin connector:
    *Pin 1 connected to Arduino GND
    *Pin 2 connected to Arduino 5V 
    *Pin 4 connected to Arduino RX (here: 10)
    *Pin 5 connected to Arduino TX (here: 9)
  - Solenoid valve on relay module, connected to digital pin on Arduino (here: 3)

  The software:
  Download SerialPlot and use the configuration file (*.ini) from the Ardoxy github repository.
  Import the settings in SerialPlot using File>>Load Settings
  Or simply read the values from the serial monitor.

  created 11 November 2021
  by Stefan Mucha

*/

#include <Ardoxy.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>

//#######################################################################################
//###                              General settings                                   ###
//#######################################################################################

// Set experimental conditions
unsigned long sampInterval = 5000;                    // sampling interval in ms (due to the duration of the measurement and communication, use interval > 1000 msec)
double airSatThreshold = 87.00;                       // target air saturation value
unsigned long experimentDuration = 20 * 60 * 1000UL;  // duration for controlled DO in ms

// Define pins
const int RX = 10;                                    // RX and TX pins for serial communication with FireStingO2
const int TX = 9;                                     // RX and TX pins for serial communication with FireStingO2
const int relayPin = 3;                               // output pin to relay module

// Define coefficients for PID control. TEST THE SETTINGS FOR PID CONTROL BEFORE YOU USE THIS SYSTEM!!   
double Kp = 10;                                       // coefficient for proportional control
double Ki = 1;                                        // coefficient for integrative control
double Kd = 1;                                        // coefficient for differential control
long int windowSize = round(sampInterval/200);        // PID controller will calculate an output between 0 and windowSize.
                                                      // This will be multiplied by 200 to ensure a minimum opening time of 200 msec to protect the relays. 
                                                      // E.g. output = 1 -> opening time 200 msec; output 50 -> opening time 10,000 msec


//#######################################################################################
//###                           Requisite variables                                   ###
//### In the following, necessary variables are defined. If you don't change too much ###
//### of this code, you will not need to change a lot here but you will need to read  ###
//### through it at least to understand how the functions work.                       ###
//#######################################################################################

// DO measurement
long DOInt, tempInt;                        // for measurement result
double DOFloat, tempFloat;                  // measurement result as floating point number
int check;                                  // numerical indicator of succesful measurement (1: success, 0: no connection, 9: mismatch)
char SeqMeasCom[7] = "SEQ 1\r";             // measurement commmand that is sent to sensor. Number indicates measurement channel
char DOReadCom[11] = "REA 1 3 4\r";         // template for DO-read command that is sent to sensor
char tempReadCom[11] = "REA 1 3 5\r";       // template for temp-read command that is sent to sensor

// Measurement timing
unsigned long loopStart, elapsed;           // ms timestamp of beginning and end of measurement loop
unsigned long progStart, progEnd;           // ms timestamp of beginning and end of experiment

// Switches and logical operators
bool startTrigger = false;                  // trigger for start of measurement
bool valveOpen = false;                     // indicator if the solenoid should remain open over one loop iteration

// PID control
double output;                              // holds output that was calculated by PID library


//#######################################################################################
//###                            Initiate libraries                                   ###
//#######################################################################################

PID valvePID(&DOFloat, &output, &airSatThreshold, Kp, Ki, Kd, REVERSE);
SoftwareSerial mySer(RX, TX);
Ardoxy ardoxy(&mySer);


//#######################################################################################
//###                                   Setup                                         ###
//#######################################################################################

void setup() {
  Serial.begin(19200);                            // Start serial communication
  ardoxy.begin(19200);                            // Start serial communication with FireSting

  // Set up PID control
  valvePID.SetMode(AUTOMATIC);
  valvePID.SetSampleTime(sampInterval);
  valvePID.SetOutputLimits(0, windowSize);

  // Declare output pins for relay operation and write HIGH to close valves (THIS MIGHT DEPEND ON YOUR VALVES)
  pinMode(relayPin, OUTPUT);
//  delay(50);
//  digitalWrite(relayPin, HIGH);

  // Print experimental conditions
  Serial.println("------------ Ardoxy measure and control example ------------");
  Serial.println("FireSting channel: 1");
  Serial.print("Measurement interval (ms): ");
  Serial.println(sampInterval);
  Serial.print(" Air saturation threshold (% air sat.): ");
  Serial.println(airSatThreshold);
  Serial.println("Use the SerialPlot software to plot DO and temperature");
  Serial.println("Send \"1\" to start measurement and \"0\" to end measurement.");
  Serial.println("------------------------------------------------------------");
}


void loop() {
  // wait for serial input to start measurement.
  if (Serial.available() > 0) {
    switch(Serial.read()){
      case '1':
          startTrigger = true;
          Serial.println("DO_air_sat;Temp_deg_C;Open_time");
          // Define time points for decrease end and trial end
          progStart = millis();
          progEnd = progStart + experimentDuration;
          break;
      case '0':
          startTrigger = false;
          digitalWrite(relayPin, HIGH);
          break;
    }
  }

  if (startTrigger) {
    loopStart = millis();                       // get time at beginning of loop

    // If the end of the experiment hasn't been reached...
    if (loopStart <= progEnd){
      check = ardoxy.measure(SeqMeasCom);       // measure sequence (ONLY FIRMWARE 3.XX on FireSting)
      if(check == 1){
        DOInt = ardoxy.readout(DOReadCom);      // read DO value from results register
        delay(20);
        tempInt = ardoxy.readout(tempReadCom);  // read temperature value from results register
        delay(20);

        // convert to floating point numbers
        DOFloat = DOInt / 1000.00;
        tempFloat = tempInt / 1000.00;
        
        // compute opening time of solenoid valve
        valvePID.Compute();

        // Print to serial
        Serial.print(DOFloat);
        Serial.print(";");
        Serial.print(tempFloat);
        Serial.print(";");
        Serial.println(round(output*200/1000));

        // operate solenoid
        if (output*200 < sampInterval) {        // if the opening time is smaller than the loop duration...
          if (valveOpen) {                      // ...if the valve was open from the previous loop, open for the calculated duration, then close
            delay(output*200);
            digitalWrite(relayPin, HIGH);
            valveOpen = false;
          }
          else {
            digitalWrite(relayPin, LOW);
            delay(output*200);
            digitalWrite(relayPin, HIGH);
          }
        }
        else {
          digitalWrite(relayPin, LOW);
          valveOpen = true;
        }



        // wait for next loop iteration
        elapsed = millis()-loopStart;
        delay(sampInterval - elapsed);          
      }
      
      // If the measurement returns with an error
      else {
        startTrigger = false;
        analogWrite(relayPin, HIGH);
        Serial.println("Com error. Check connections and send \"1\" to restart.");
      }
    }
    else {
      analogWrite(relayPin, HIGH);
      Serial.println("End of experiment. Arduino stopps. Send \"1\" to re-start.");
      startTrigger = false;
    }
  }
}
