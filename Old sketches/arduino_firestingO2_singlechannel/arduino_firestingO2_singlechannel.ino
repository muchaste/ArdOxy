//#######################################################################################
//###           ArdOxy - Arduino based dissolved oxygen control system                ###
//### This sketch lets you to measure 1 dissolved oxygen and temperature channel via  ###
//### a FireStingO2 sensor                                                            ###
//### Core hardware components:                                                       ###
//### Arduino UNO R3 (1x), relay module, PyroScience FirestingO2, solenoid valve      ###
//### -------------- Read the documentation before using this sketch!---------------- ###
//### Adapt this sketch to fit your application and carefully test the system before  ###
//### using it.                                                                       ###
//### Low oxygen is potentially harmful, so use this sketch carefully and only under  ###
//### monitored conditions. No liability for damages caused by misuse of this sketch  ###
//### is taken ~Stefan Mucha, 2019                                                    ###
//#######################################################################################

#include <PID_v1.h>
#include <SoftwareSerial.h>

//#######################################################################################
//###                              General settings                                   ###
//#######################################################################################

long int samples = 1;                                                                           // number of measurements that are averaged to one air saturation value 
                                                                                                // to reduce sensor fluctuation (oversampling)
long interval = 30 * 1000UL;                                                                     // measurement and control interval in second
double airSatThreshold = 400.00;                                                                  // air saturation to be kept in % air sat * 1000                          
unsigned long decrTime = 30*60*1000L;                                                           // timespan for DO decrease
unsigned long contTime = 30*60*1000L;                                                            // timespan for continuous DO at airSatThreshold
SoftwareSerial mySer(10, 11); //RX TX


//#######################################################################################
//###                                Relay settings                                   ###
//### The arduino operates solenoid valves through a relay module to bubble N2 into   ###
//### the tanks. It calculates a time interval using the PIDlibrary (by Brett         ###
//### Beauregard). TEST THE SETTINGS FOR PID CONTROL BEFORE YOU USE THIS SYSTEM!!     ###
//### If you're not sure, replace the PID control with a simpler solution             ###
//### (e.g. bubble N2 for 10 sec if there's a large oxygen difference etc.).          ###
//### To control for stress due to bubbling, compressed air is bubbled into control   ###
//### tanks.                                                                          ###
//#######################################################################################

int relayPin = 12;          // pins for relay operation ***ADAPT THIS TO FIT YOUR WIRING***
double Kp = 10;             // coefficient for proportional control
double Ki = 1;              // coefficient for integrative control
double Kd = 1;              // coefficient for differential control
long int WindowSize = 25;                                       // The PID will calculate an output between 0 and 25.
                                                                // This will be multiplied by 200 to ensure a minimum opening time of 200 msec. 
                                                                // E.g. output = 1 -> opening time 200 msec; output 25 -> opening time 5,000 msec


//#######################################################################################
//###                           Requisite variables                                   ###
//### In the following, necessary variables are defined. If you don't change too much ###
//### of this code, you will not need to change a lot here but you will need to read  ###
//### through it at least to understand how the functions work.                       ###
//#######################################################################################

//# Switches and logical operators #
boolean newData = false;                      // true if serial data was received from sensors
boolean emptyBuffer = false;                  // true if buffer for serial data is empty

//# Measurement timing #
unsigned long progStart;
unsigned long decrStart;
unsigned long decrEnd;
unsigned long contEnd;

unsigned long startTime;                      // time taken at start of loop
unsigned long endTime;                        // time taken at end of loop
unsigned long elapsed;                        // elapsed time between end- and start-time

//# Oxygen optode #
char DOMeasCom[7] = "SEQ1\r";                 // measurement commmand that is sent to sensor during void toggleMeasurement(). Number indicates measurement channel
char DOReadCom[11] = "REA1 3 4\r";            // template for DO-read command that is sent to sensor during void toggleRead() (length = 10 because of /0 string terminator)
const byte numChars = 30;                     // array length for incoming serial data (longer than needed)
char receivedChars[numChars];                 // array to store the incoming serial data (used in receiveData() )
char bufferChars[numChars];                   // array to store incoming data that is not needed (used to clear buffer() )
int len;                                      // length of actually received serial data
char valueStr[20];                            // array to hold only readings from receivedChars (longer than needed)
long DOInt;                                   // variable for readings that are parsed from valueStr
double airSatFloat;                           // array for floating point value of air saturation for datalogging, display etc.
double startDO;                                 // variable for first DO value
double deltaDO;                                 // variable for DO change per measurement interval
double changeDO;
double lastDO;

//# Relay operation #
double relayArray[3];                         // array with relay pin and assigned output values
double Output;                                // holds output that was calculated by PID library
double openTime;

//PID setup
PID relay1PID(&changeDO, &Output, &deltaDO, Kp, Ki, Kd, REVERSE);    


//#######################################################################################
//###                           Requisite Functions                                   ###
//###               These functions are executed in the main() loop.                  ###
//### The functions for receiving serial data, parsing it and clearing the buffer are ###
//### modified from the excellent post on the Arduino Forum by user Robin2 about the  ###
//### basics of serial communication (http://forum.arduino.cc/index.php?topic=396450).###
//#######################################################################################

//# Send the commands to toggle measurement to and readout values from the FireStingO2 device via Serial #
void toggleDOMeasurement(char command[7]) {
  newData = false;
  emptyBuffer = false;
  static byte ndx = 0;
  char endMarker = '\r';
  char rc;
  mySer.write(command);
  mySer.flush();
  delay(700);
  // Clear echoed measurement command from serial buffer
  while (mySer.available() > 0 && emptyBuffer == false) {
    delay(2);
    rc = mySer.read();
    if (rc != endMarker) {
      bufferChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      bufferChars[ndx] = '\0';  // terminate the string
      ndx = 0;
      emptyBuffer = true;
    }
  }
}

//# Receive serial data (modified from http://forum.arduino.cc/index.php?topic=396450) #
int receiveDOData(char command[11]) {                                                                    // receives serial data and stores it in array until endmarker is received
  static byte ndx = 0;                                                                  // index for storing in the array
  char endMarker = '\r';                                                                // declare the character that marks the end of a serial transmission
  char rc;                                                                              // temporary variable to hold the last received character
  mySer.write(command);
  mySer.flush();
  delay(300);
  while (mySer.available() > 0 && newData == false && emptyBuffer == true) {        // only read serial data if the buffer was emptied before and it's new data
    delay(2);                                                                        
    rc = mySer.read();
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
      newData = true;                                                                 // new data has been received and is in the receivedChars buffer
    }
  }
  len = strlen(receivedChars)+1;                // get length of received string + termination \0
  len = len - 9;                                // subtract the length of the echoed read command - I determined this by testing it, there might be a smarter way to do this
  for (int k = 0; k < len; k++) {               // store the last digits of string in new character array valueStr, starting at and including position 10. This is necessary as
    valueStr[k] = receivedChars[(9 + k)];       // the sensor echoes the readout command, e.g. "REA1 3 4" before the air saturation values
  }
  DOInt = atol(valueStr);                       // parse the character to integers - the air saturation values are given as [% air saturation x 1000], temperature as [°C x 1000]
  return DOInt;
}


//# Toggle relay based on measured airSat values #
void toggleRelay() {
  relay1PID.Compute();                                  // compute the output (output * 200 = opening time) based on the input (air saturation) and threshold
  openTime = double(int(Output)*200.00);           // PID computes an output between 0 and 50, the multiplicator makes sure that the relay operation time is at least 200ms
  if (openTime > 0){                               // skip the channels that don't have to be operated (output = 0.00)
    digitalWrite(relayPin, LOW);
    delay(openTime);
    digitalWrite(relayPin, HIGH);
  }
}

//# Clear all serial buffers (modified from http://forum.arduino.cc/index.php?topic=396450) #
void clearAllBuffers() {                                                                    // similar to receiveData(), clears serial buffer from echoes
  static byte ndx = 0;
  char endMarker = '\r';
  char rc;
  emptyBuffer = false;
  while (mySer.available() > 0 && emptyBuffer == false) {
    delay(2);
    rc = mySer.read();
    if (rc != endMarker) {
      bufferChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      bufferChars[ndx] = '\0';  // terminate the string
      ndx = 0;
      emptyBuffer = true;
    }
  }
}

//# Reboot the Arduino #
void(* resetFunc) (void) = 0;                         // function to restart the Arduino when a communication error occurs




//#######################################################################################
//###                                   Setup                                         ###
//#######################################################################################

void setup() {
  Serial.begin(19200);
  mySer.begin(19200);
  
//# Declare output pins for relay operation #
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);
  
//# Define time points for decrease end and trial end
  progStart = millis();
  decrEnd = progStart + decrTime;
  contEnd = decrEnd + contTime;

//# Measure DO once
  toggleDOMeasurement(DOMeasCom);
  delay(700);
  receiveDOData(DOReadCom);
  startDO = DOInt/1000.00;
  deltaDO = (startDO-airSatThreshold)/decrTime;
  lastDO = startDO;
  Serial.print("DO: ");
  Serial.println(startDO);
  delay(500);
  Serial.print("Decrease rate per minute: ");
  Serial.print(deltaDO);
  delay(1000);

//# Set up PID control #
  relay1PID.SetMode(AUTOMATIC);
  relay1PID.SetSampleTime(interval);
  relay1PID.SetOutputLimits(0, WindowSize);

  startTime = millis();                                       // start timer of loop
}

//#######################################################################################
//###                                 Main loop                                       ###
//###               Congratulations, you made it to the main loop.                    ###
//### I inserted delays between every subfunctions to ensure their completion before  ###
//### the program moves to the next step. Some of these are crucial as the sensor     ###
//### needs time to complete measurements.                                            ###
//#######################################################################################

void loop() {
  while(startTime < contEnd){
    startTime = millis();                                       // start timer of loop
    DOInt = 0;                                                // reset value variable
    receivedChars[0] = (char)0;                               // empty char arrays
    bufferChars[0] = (char)0;                                   
    toggleDOMeasurement(DOMeasCom);
    delay(700);
    receiveDOData(DOReadCom);
    if (strncmp(DOReadCom, receivedChars, 9) == 0){
      airSatFloat = DOInt / 1000.00;                            // create floating point number for logging, display, etc. Results in 0 if there's a communication error
      changeDO = airSatFloat - lastDO;
      lastDO = airSatFloat;
    }
    else {                                                      // reboot system if communication error occurs
      resetFunc();
    }
    if (startTime > decrEnd){
      deltaDO = airSatThreshold;
      changeDO = DOInt;
    }
    if (newData == true && emptyBuffer == true) {
      Serial.println(airSatFloat);
      toggleRelay();                                            // operate relays to open solenoid valves
      endTime = millis();
      elapsed = endTime - startTime;                            // measure duration of loop
      delay(interval - elapsed);                               // adjust delay so that loop duration equals measurement interval
    }
  }
}
