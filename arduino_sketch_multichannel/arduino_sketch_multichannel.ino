//#######################################################################################
//###           ArdOxy - Arduino based dissolved oxygen control system                ###
//### Due to my limited coding skills, this sketch is partially hardcoded to measure  ###
//### 8 dissolved oxygen channels via 2 FireStingO2 sensors. If you want to use fewer ###
//### channels/sensors, you have to adapt this sketch accordingly. Especially the     ###
//### relay PID control setup is hardcoded.                                           ###
//### Core hardware components:                                                       ###
//### Arduino MEGA 2560 (1x), Adafruit Data logging shield (ID 1141),                 ###
//### Adafruit LCD shield (ID 714), SainSmart 8 channel relay module (101-70-102),    ###
//### PyroScience FirestingO2 (2x), solenoid valves (8x)                              ###
//### -------------- Read the documentation before using this sketch!---------------- ###
//### Adapt this sketch to fit your application and carefully test the system before  ###
//### using it.                                                                       ###
//### Low oxygen is potentially harmful, so use this sketch carefully and only under  ###
//### monitored conditions. No liability for damages caused by misuse of this sketch  ###
//### is taken ~Stefan Mucha, 2019                                                    ###
//#######################################################################################

#include <PID_v1.h>
#include "SD.h"
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#define WHITE 0x7                                     // LCD color code


//#######################################################################################
//###                              General settings                                   ###
//### This program is intended to work with 8 channels for measurement and control.   ###
//### Thus most arrays have a length of 8. Adapt the following lines to your use.     ###
//#######################################################################################

const int s1ChannelNumber = 4;                                                          // number of used channels on sensor 1
const int s2ChannelNumber = 4;                                                          // number of used channels on sensor 2
const int channelNumber = s1ChannelNumber + s2ChannelNumber;                            // total number of measurement channels
long int samples = 1;                                                                   // number of measurements that are averaged to one air saturation value 
                                                                                        // to reduce sensor fluctuation (oversampling)
char tankID[8][6] = {"B4", "C3", "B1", "C8", "E7", "E4", "E1", "D6"};                   // IDs assigned to the channels in the order of the channelArray
char tempID[2][6] = {"B4", "E4"};                                                       // tanks where the temperature sensors are placed (1 per sensor)
long interval = 30 * 1000UL;                                                            // measurement and control interval in second
double airSatThreshold[8] = {100.0, 100.0, 100.0, 15.0, 100.0, 15.0, 15.0, 15.0};       // air saturation threshold including first decimal                            
const int chipSelect = 10;                                                              // chip pin for SD card (UNO: 4; MEGA: 53, Adafruit shield: 10)
double lowDOThreshold = 7.0;                                                            // threshold for low oxygen that causes the program to do something
int channelArray[channelNumber] = {1, 2, 3, 4, 1, 2, 3, 4};                             // measurement channels from firesting devices 1 and 2 in that order



//#######################################################################################
//###                                Relay settings                                   ###
//### The arduino operates solenoid valves through a relay module to bubble N2 into   ###
//### the tanks. It calculates a time interval using the PIDlibrary (by Brett         ###
//### Beauregard). TEST THE SETTINGS FOR PID CONTROL BEFORE YOU USE THIS SYSTEM!!     ###
//### If you're not sure, replace the PID control with a simpler solution      ###
//### (e.g. bubble N2 for 10 sec if there's a large oxygen difference etc.).          ###
//#######################################################################################

int relayPin[2][8] = {                              // pins for relay operation ***ADAPT THIS TO FIT YOUR WIRING***
  {22, 24, 26, 28, 30, 32, 34, 36},                 // outputs to experimental tanks
  {23, 25, 27, 29, 31, 33, 35, 37}                  // output copies to control tanks that receive compressed air
};     
double Kp[8] = {10, 10, 10, 10, 10, 10, 10, 10};    // coefficient for proportional control
double Ki[8] = {1, 1, 1, 1, 1, 1, 1, 1};            // coefficient for integrative control
double Kd[8] = {1, 1, 1, 1, 1, 1, 1, 1};            // coefficient for differential control
long int WindowSize = 75;                           // The PID will calculate an output between 0 and 100.
                                                    // This will be multiplied by 200 to ensure a minimum opening time of 200 msec. 
                                                    // E.g. output = 1 -> opening time 200 msec; output 50 -> opening time 10,000 msec



//#######################################################################################
//###                           Requisite variables                                   ###
//### In the following, necessary variables are defined. If you don't change too much ###
//### of this code, you will not need to change a lot here but you will need to read  ###
//### through it at least to understand how the functions work.                       ###
//#######################################################################################

//# Switches and logical operators #
boolean lowDO = false;                        // boolean for low oxygen threshold
int activeSensor = 1;                         // index to switch between sensors (if s2ChannelNumber = 0 it stays at 1)
int activeChannel = 1;                        // measurement channel
boolean newData = false;                      // true if serial data was received from sensors
boolean emptyBuffer = false;                  // true if buffer for serial data is empty
boolean comCheck = false;                     // true if echoed command matches sent command

//# Measurement timing #
unsigned long startTime;                      // time taken at start of loop
unsigned long endTime;                        // time taken at end of loop
unsigned long elapsed;                        // elapsed time between end- and start-time

//# Logging and SD card #
RTC_PCF8523 RTC;                              // real time clock
File logfile;                                 // initializes the logfile
char filename[16];                            // array for filename of .csv file
int fn = 0;                                   // filename index for .csv file
int n = 0;                                    // row index for logging

//# Oxygen optode #
char toggleMeasString[6] = "SEQ 1";           // measurement commmand that is sent to sensor during void toggleMeasurement(). SEQ triggers a sequence of measurements 
                                              // (humidity, temperature, pressure, air saturation). This way, the air saturation values will be compensated automatically
                                              // for fluctuations in humidity, temperature and pressure.
char toggleDORead[10] = "REA 1 3 4";          // template for DO-read command that is sent to sensor during void toggleRead() (length = 10 because of /0 string terminator)
char toggleTempRead[10] = "REA 1 3 5";        // temperature read command
const byte numChars = 30;                     // array length for incoming serial data (longer than needed)
char receivedChars[numChars];                 // array to store the incoming serial data (used in receiveData() )
char bufferChars[numChars];                   // array to store incoming data that is not needed (used in clearBuffer() )
int len;                                      // length of actually received serial data
char valueStr[20];                            // array to hold only readings from receivedChars (longer than needed)
long valueInt;                                // variable for readings that are parsed from valueStr
long airSatSum;                               // summing variable for air saturations in case oversampling is used (see sample variable)
double airSatFloat[channelNumber];                        // array for floating point value of air saturation for datalogging, display etc.
double lowDOValue;                            // variable for low DO values that are below the critical threshold defined above
double tempFloat[2];                          // array to store temperature readings for logging

//# Relay operation #
double relayArray[4][channelNumber];                      // array with relay pin and assigned output values
double Output[8];                             // holds output that was calculated by PID library

//Hardcoded PID setup - there's probably a way to set up only those that are needed but I can only think of a long chain of if-and conditions
PID relay1PID(&airSatFloat[0], &Output[0], &airSatThreshold[0], Kp[0], Ki[0], Kd[0], REVERSE);    
PID relay2PID(&airSatFloat[1], &Output[1], &airSatThreshold[1], Kp[1], Ki[1], Kd[1], REVERSE);
PID relay3PID(&airSatFloat[2], &Output[2], &airSatThreshold[2], Kp[2], Ki[2], Kd[2], REVERSE);
PID relay4PID(&airSatFloat[3], &Output[3], &airSatThreshold[3], Kp[3], Ki[3], Kd[3], REVERSE);
PID relay5PID(&airSatFloat[4], &Output[4], &airSatThreshold[4], Kp[4], Ki[4], Kd[4], REVERSE);
PID relay6PID(&airSatFloat[5], &Output[5], &airSatThreshold[5], Kp[5], Ki[5], Kd[5], REVERSE);
PID relay7PID(&airSatFloat[6], &Output[6], &airSatThreshold[6], Kp[6], Ki[6], Kd[6], REVERSE);
PID relay8PID(&airSatFloat[7], &Output[7], &airSatThreshold[7], Kp[7], Ki[7], Kd[7], REVERSE);

//# LCD Display #
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
int airSatLCD = 0;                                      // integer to display rounded values (due to space constraints on the LCD)


//#######################################################################################
//###                           Requisite Functions                                   ###
//###               These functions are executed in the main() loop.                  ###
//### The functions for receiving serial data, parsing it and clearing the buffer are ###
//### modified from the excellent post on the Arduino Forum by user Robin2 about the  ###
//### basics of serial communication (http://forum.arduino.cc/index.php?topic=396450).###
//#######################################################################################

//# Send the commands to toggle measurement to and readout values from the FireStingO2 devices via Serial #
//# I copied these functions to switch serial ports based on the active sensor. There's probably a better way to do this
void toggleMeasurement(char command[6]) {
  if (activeSensor == 1) {
    Serial1.write(command);
    Serial1.write('\r');
    emptyBuffer = false;
    Serial1.flush();
  }
  else if (activeSensor == 2) {
    Serial2.write(command);
    Serial2.write('\r');
    emptyBuffer = false;
    Serial2.flush();
  }
}

void toggleRead(char command[10]) {
  if (activeSensor == 1) {
    Serial1.write(command);
    Serial1.write('\r');
    Serial1.flush();
  }
  else if (activeSensor == 2) {
    Serial2.write(command);
    Serial2.write('\r');
    Serial2.flush();
  }
}


//# Receive serial data (modified from http://forum.arduino.cc/index.php?topic=396450) #
void receiveData() {                                                                    // receives serial data and stores it in array until endmarker is received
  static byte ndx = 0;                                                                  // index for storing in the array
  char endMarker = '\r';                                                                // declare the character that marks the end of a serial transmission
  char rc;                                                                              // temporary variable to hold the last received character
  if (activeSensor == 1) {
    while (Serial1.available() > 0 && newData == false && emptyBuffer == true) {        // only read serial data if the buffer was emptied before and it's new data
      delay(2);                                                                        
      rc = Serial1.read();
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
        comCheck = false;                                                               // the incoming string has not yet been checked with the sent string
      }
    }
  }
  else if (activeSensor == 2) {                                                         // same as above for the second sensor
    while (Serial2.available() > 0 && newData == false && emptyBuffer == true) {
      delay(2);
      rc = Serial2.read();
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0';
        ndx = 0;
        newData = true;
        comCheck = false;
      }
    }
  }
}

//# Clear the serial buffer (modified from http://forum.arduino.cc/index.php?topic=396450) #
void clearBuffer() {                                                                    // similar to receiveData(), clears serial buffer from echoes
  static byte ndx = 0;
  char endMarker = '\r';
  char rc;
  if (activeSensor == 1) {
    while (Serial1.available() > 0 && emptyBuffer == false) {
      delay(2);
      rc = Serial1.read();
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
        newData = false;
      }
    }
  }
  else if (activeSensor == 2) {
    while (Serial2.available() > 0 && emptyBuffer == false) {
      delay(2);
      rc = Serial2.read();
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
        newData = false;
      }
    }
  }
}

//# Extract air saturation and temperature values from received serial data #
int extractValue() {            
  len = strlen(receivedChars)+1;                  // get length of received string + termination \0
  len = len - 10;                                 // subtract the length of the echoed read command - I determined this by testing it, there might be a smarter way to do this
  for (int i = 0; i < len; i++) {                 // store the last digits of string in new character array valueStr, starting at and including position 10. This is necessary as
    valueStr[i] = receivedChars[(10 + i)];        // the sensor echoes the readout command, e.g. "REA 1 3 4" before the air saturation values
  }
  valueInt = atol(valueStr);                      // parse the character to integers - the air saturation values are given as [% air saturation x 1000], temperature as [°C x 1000]
  return valueInt;
}

//# Send air saturation readings to serial monitor and LCD (sets newData to false) #
void showNewData() {
  if (newData == true && emptyBuffer == true) {
    lcd.clear();
    lcd.setCursor(0, 0);
    DateTime now;
    now = RTC.now();  
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" - ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    for (int i = 0; i < 2; i++){
      Serial.print(tempID[i]);
      Serial.print(": ");
      Serial.print(tempFloat[i]);
      Serial.println("°C ");
    }
    for (int i = 0; i < (channelNumber); i++) {
      if (i == 4) {                                     
        lcd.setCursor(0, 1);                          // break line on LCD display when the 5th DO value is reached
      }
      airSatLCD = int(lround(airSatFloat[i]));
      lcd.print(airSatLCD);
      lcd.print(" ");
      Serial.print(tankID[i]);
      Serial.print(": ");
      Serial.print(airSatFloat[i]);
      Serial.println("% air saturation");
    }
    newData = false;
  }
}

//# Check if air saturations are below the security threshold #
void DOCheck() {
  for (int i = 0; i < channelNumber; i++){
    if (airSatFloat[i] < lowDOThreshold){
      lowDO = true;
      lowDOValue = airSatFloat[i];
    }
  }
}

//### Toggle relay based on measured airSat values ###
void toggleRelay() {
  relay1PID.Compute();                                  // compute the output (output * 200 = opening time) based on the input (air saturation) and threshold
  relay2PID.Compute();
  relay3PID.Compute();
  relay4PID.Compute();
  relay5PID.Compute();
  relay6PID.Compute();
  relay7PID.Compute();
  relay8PID.Compute();
  for (int i = 0; i < channelNumber; i++) {
    relayArray[0][i] = relayPin[0][i];
    relayArray[1][i] = relayPin[1][i];
    relayArray[2][i] = double(int(Output[i])*200.00);   // PID computes an output between 0 and 50, the multiplicator makes sure that the relay operation time is at least 200ms
    relayArray[3][i] = airSatFloat[i];
  }
  double temp[4];                                       // temporary array to sort all channels with the smallest based on the computed output (lowest output first). 
                                                        // This is necessary as the valves are kept open using the delay()-function which halts all activity. 
                                                        // With 8 channels, opening one valve after the other is too time consuming. With the opening times sorted, all valves 
                                                        // can be opened at the same time and then closed one after another.
  for (int i = 0; i < channelNumber - 1; i++) {         // sort array with lowest difference first
    for (int j = i + 1; j < channelNumber; j++) {
      if (relayArray[2][j] < relayArray[2][i]) {
        temp[0] = relayArray[0][i];                     // store higer value + relayPin in temp array
        temp[1] = relayArray[1][i];
        temp[2] = relayArray[2][i];
        temp[3] = relayArray[3][i];
        relayArray[0][i] = relayArray[0][j];            // move lower value to position i
        relayArray[1][i] = relayArray[1][j];
        relayArray[2][i] = relayArray[2][j];
        relayArray[3][i] = relayArray[3][j];
        relayArray[0][j] = temp[0];                     // insert higher value at position j
        relayArray[1][j] = temp[1];                     // now the values at i and j have switched places in the relayArray
        relayArray[2][j] = temp[2];
        relayArray[3][j] = temp[3];
      }
    }
  }
  for (int i = 0; i < channelNumber; i++) {
    if (relayArray[2][i] > 0){                              // skip the channels that don't have to be operated (output = 0.00)
      for (int j = i; j < channelNumber; j++) {             // open all other valves
        digitalWrite(relayArray[0][j], LOW);
        digitalWrite(relayArray[1][j], LOW);
      }
      if (i == 0) {                                         // close the valve with the lowest output first
        delay(relayArray[2][i]);
        digitalWrite(relayArray[0][i], HIGH);
        digitalWrite(relayArray[1][i], HIGH);
      }
      else {
        delay(relayArray[2][i] - relayArray[2][i - 1]);     // keep the other valves open based on the difference of the output value
        digitalWrite(relayArray[0][i], HIGH);               // close the valve
        digitalWrite(relayArray[1][i], HIGH);
      }
    }
  }
}

//### Log dissolved oxygen measurements to SD card ###
void writeToSD() {
  if (!SD.exists(filename)) {
    Serial.println("error: can't read SD");
    while (1);
  }
  else {
    DateTime now;
    now = RTC.now();                                  // fetch time and date from RTC
    n = n + 1;                                        // increase n by one for each measurement
    logfile.print(n);                                 // print row index n
    logfile.print(";");                               // print a semicolon to separate values
    logfile.print(now.year(), DEC);                   // print date and time of measurement
    logfile.print("/");
    logfile.print(now.month(), DEC);
    logfile.print("/");
    logfile.print(now.day(), DEC);
    logfile.print(";");
    logfile.print(now.hour(), DEC);
    logfile.print(":");
    logfile.print(now.minute(), DEC);
    logfile.print(":");
    logfile.print(now.second(), DEC);
    logfile.print(";");
    for (int i = 0; i < 2; i++){
      logfile.print(tempFloat[i]);
      logfile.print(";");
    }
    for (int i = 0; i < channelNumber; i++) {         // print air saturation measurements for each channel
      logfile.print(airSatFloat[i]);
      logfile.print(";");
    }
    logfile.println();
    logfile.flush();                                  // save data to logfile
  }
}

void(* resetFunc) (void) = 0;                         // function to restart the Arduino when a communication error occurs


//#######################################################################################
//###                                   Setup                                         ###
//### You may want to get rid of all the status updates that are printed on the LCD   ###
//### and the serial monitor. I use them to inspect what the settings are.            ###
//#######################################################################################

void setup() {
  Serial.begin(19200);
  Serial.println("Automated oxygen control system booting ... ");
  Serial1.begin(19200);
  if (s2ChannelNumber > 0) {
    Serial2.begin(19200);
  }
    
//# Set up one PID per channel #
  relay1PID.SetMode(AUTOMATIC);
  relay1PID.SetSampleTime(interval);
  relay1PID.SetOutputLimits(0, WindowSize);
  relay2PID.SetMode(AUTOMATIC);
  relay2PID.SetSampleTime(interval);
  relay2PID.SetOutputLimits(0, WindowSize);
  relay3PID.SetMode(AUTOMATIC);
  relay3PID.SetSampleTime(interval);
  relay3PID.SetOutputLimits(0, WindowSize);
  relay4PID.SetMode(AUTOMATIC);
  relay4PID.SetSampleTime(interval);
  relay4PID.SetOutputLimits(0, WindowSize);
  relay5PID.SetMode(AUTOMATIC);
  relay5PID.SetSampleTime(interval);
  relay5PID.SetOutputLimits(0, WindowSize);
  relay6PID.SetMode(AUTOMATIC);
  relay6PID.SetSampleTime(interval);
  relay6PID.SetOutputLimits(0, WindowSize);
  relay7PID.SetMode(AUTOMATIC);
  relay7PID.SetSampleTime(interval);
  relay7PID.SetOutputLimits(0, WindowSize);
  relay8PID.SetMode(AUTOMATIC);
  relay8PID.SetSampleTime(interval);
  relay8PID.SetOutputLimits(0, WindowSize);

//# Start LCD display, clear serial buffer #
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DO ctrl booting.");
  lcd.setCursor(0, 1);
  lcd.print("Empty buffer");
  clearBuffer();
  delay(100);

//# Declare output pins for relay operation #
  lcd.clear();
  lcd.print("Relay pins..");
  for (int i = 0; i < channelNumber; i++) {       // declare relay pins as output pins and write HIGH to close magnetic valves
    pinMode(relayPin[0][i], OUTPUT);
    pinMode(relayPin[1][i], OUTPUT);
    digitalWrite(relayPin[0][i], HIGH);
    digitalWrite(relayPin[1][i], HIGH);
  }
  delay(100);

//# Initialize the real time clock #
  lcd.clear();
  lcd.print("Init RTC...");
  Wire.begin();                                   // initialize real time clock
  if (!RTC.begin()) {                             // check if RTC is initialized
    lcd.setCursor(0, 1);
    Serial.println("RTC failed");
    lcd.print("RTC failed");
    while(1);
  }
  //RTC.adjust(DateTime(F(__DATE__), F(__TIME__))); // set rtc to the time this sketch was compiled - ONLY NECESSARY ON FIRST UPLOAD, THEN COMMENT THE LINE OUT AND RE-UPLOAD
  delay(100);
  lcd.clear();
  DateTime now;
  now = RTC.now();                                // fetch time from RTC to display
  lcd.print(now.year(), DEC);
  lcd.print('/');
  lcd.print(now.month(), DEC);
  lcd.print('/');
  lcd.print(now.day(), DEC);
  lcd.setCursor(0,1);
  lcd.print(now.hour(), DEC);
  lcd.print(':');
  lcd.print(now.minute(), DEC);
  lcd.print(':');
  lcd.print(now.second(), DEC);
  delay(3000);                                    // long delay to check the date on the display

//# Initialize SD card #
  lcd.clear();
  lcd.print("Check SD...");
  if (!SD.begin(chipSelect)) {                    // check if SD card is present and can be read
    Serial.println("Card failed, or not present. Please insert SD card and reboot the system.");
    lcd.setCursor(0, 1);
    lcd.print("SD failed");
    while (1);
  }
  delay(500);

//# Create a new logfilename #
  lcd.clear();
  lcd.print("Create .csv...");
  snprintf(filename, sizeof(filename), "dolog%03d.csv", fn); // choose filename for logfile on SD that does not exist yet, includes a three-digit sequence number in the file name
  while (SD.exists(filename)) {
    fn++;
    snprintf(filename, sizeof(filename), "dolog%03d.csv", fn);
  }
  logfile = SD.open(filename, FILE_READ);
  logfile.close();                                          // now filename[] contains the name of a file that doesn't exist
  lcd.setCursor(0,1);
  lcd.print(filename);
  delay(500);
  
//# Create logfile #
  logfile = SD.open(filename, FILE_WRITE);                  // create logfile and write header information
  if (logfile) {
    logfile.println(";");                                   // print a leading blank line
    logfile.print("Date:;");                                // print header information: date, time, air saturation threshold, channels, tank IDs etc
    logfile.print(now.year(), DEC);
    logfile.print("/");
    logfile.print(now.month(), DEC);
    logfile.print("/");
    logfile.print(now.day(), DEC);
    logfile.print(";");
    logfile.print("Time:;");
    logfile.print(now.hour(), DEC);
    logfile.print(":");
    logfile.print(now.minute(), DEC);
    logfile.print(":");
    logfile.println(now.second(), DEC);
    logfile.print(";Measurement interval [sec]:;");
    logfile.print(interval / 1000);
    logfile.print(";Active channels:;");
    logfile.print(channelNumber);
    logfile.print(";Temp Sensor 1:;");
    logfile.print(tempID[0]);
    logfile.print(";Temp Sensor 2:;");
    logfile.println(tempID[1]);
    logfile.print("Air sat threshold [% air saturation]:;");
    for (int i = 0; i < (channelNumber); i++) {
      logfile.print(airSatThreshold[i]);
      logfile.print(";");
    }
    logfile.println();
    logfile.print("Tank ID:;");
    for (int i = 0; i < channelNumber; i++) {
      logfile.print(tankID[i]);
      logfile.print(";");
    }
    logfile.println(";");
    logfile.print("Channel:;");
    for (int i = 0; i < (channelNumber); i++) {
      logfile.print(channelArray[i]);
      logfile.print(";");
    }
    logfile.println(";");
    logfile.println(";");
    logfile.print("Measurement;Date;Time;Temp.");                  // header row for measurements: Measurement, Date, Time, tankID1, tankID2,...
    logfile.print(tempID[0]);
    logfile.print(";Temp.");
    logfile.print(tempID[1]);
    logfile.print(";");
    for (int i = 0; i < (channelNumber); i++) {
      logfile.print(tankID[i]);
      logfile.print(";");
    }
    logfile.println();
  }
  else if (!logfile) {                                        // check if logfile was created
    Serial.println("error: couldn't create logfile");
    lcd.setCursor(0, 1);
    lcd.print(".csv failed");
    while (1);                                                // do nothing
  }
  delay(500);

//# Display control settings 
  Serial.print("Start of measurement cycles. Measurement interval set at ");
  Serial.print(interval / 1000);
  Serial.println(" seconds.");
  lcd.clear();
  lcd.print("Ready");
  lcd.setCursor(0, 1);
  lcd.print("Interval: ");
  lcd.print(interval / 1000);
  lcd.print("s");
  delay(2000);
  
  lcd.clear();
  lcd.print("---Thresholds---");
  delay(1000);
  lcd.clear();
  for (int i = 0; i < channelNumber; i++){
    if (i == 2){
      lcd.setCursor(0, 1);
    }
    else if (i == 4){
      delay(2000);
      lcd.clear();
    }
    else if (i == 6){
      lcd.setCursor(0, 1);
    }
    lcd.print(airSatThreshold[i]);
    lcd.print(" ");
  }
  delay(2000); 
  lcd.clear();
  
}

//#######################################################################################
//###                                 Main loop                                       ###
//###               Congratulations, you made it to the main loop.                    ###
//### I inserted delays between every subfunctions to ensure their completion before  ###
//### the program moves to the next step. Some of these are crucial as the sensor     ###
//### needs time to complete measurements. Also with serial communication, we want to ###
//### make sure that everything has been completely sent and received.                ###
//#######################################################################################

void loop() {
  startTime = millis();                                       // start timer of loop
  if (millis() >= 86400000){
    lcd.clear();
    lcd.print("Init. reset");
    delay(10000);                                             // long delay to upload a new sketch in case of reset loop
    resetFunc();                                              // reset Arduino every 24 h
  }
  for (int i = 0; i < channelNumber; i++) {                   // loop that iterates through every channel
    airSatSum = 0;                                            // reset summing variable for measurements
    valueInt = 0;                                             // reset value variable
    receivedChars[0] = (char)0;                               // empty char arrays
    bufferChars[0] = (char)0;                                   
    if (i < s1ChannelNumber) {                                // switch sensor based on measurement channel
      activeSensor = 1;
    }
    else if (i == s1ChannelNumber) {
      activeSensor = 2;
    }  
    DateTime now;
    now = RTC.now();  
    delay(100); 
    activeChannel = channelArray[i];                          // declare channel to be measured for serial commands
    sprintf(toggleMeasString, "SEQ %d", activeChannel);       // insert channel in measurement command
    sprintf(toggleDORead, "REA %d 3 4", activeChannel);       // insert channel in readout command
    for (int j = 0; j < samples; j++) {                       // oversampling loop 
      toggleMeasurement(toggleMeasString);
      delay(1000);                                            // delay to allow the sensor to complete measurement
      clearBuffer();
      delay(100);
      if (activeChannel == 1){
        toggleRead(toggleTempRead);
        delay(300);
        receiveData();
        if (strncmp(toggleTempRead, receivedChars, 9) == 0){  // compare first 9 characters of incoming string with sent string
          comCheck = true;
        }
        if (comCheck == true){
          extractValue();
          tempFloat[activeSensor-1] = valueInt / 1000.00;
          newData = false;
        } 
        else if (comCheck == false){                          // reboot system if communication error occurs
          DateTime now;
          now = RTC.now();  
          Serial.print(now.year(), DEC);
          Serial.print('/');
          Serial.print(now.month(), DEC);
          Serial.print('/');
          Serial.print(now.day(), DEC);
          Serial.print(" - ");
          Serial.print(now.hour(), DEC);
          Serial.print(':');
          Serial.print(now.minute(), DEC);
          Serial.print(':');
          Serial.print(now.second(), DEC);
          Serial.println();
          Serial.println("Communication error!");
          resetFunc();
        }
      }
      toggleRead(toggleDORead);
      delay(300);
      receiveData();
      if (strncmp(toggleDORead, receivedChars, 9) == 0){
        comCheck = true;
      }
      if (comCheck == true){
        extractValue();
        delay(100);
        airSatSum = airSatSum + valueInt;                     // sum up air saturation readings for consecutive samples
      }
    }
    airSatFloat[i] = airSatSum / (samples * 1000.00);         // create floating point number for logging, display, etc. Results in 0 if there's a communication error
  }
  if (comCheck == true){
    showNewData();                                            // display measurement on LCD
    delay(100);
    writeToSD();                                              // log to SD card
    delay(100);
    DOCheck();                                                // check if low DO threshold is crossed
    if (lowDO == true){
      Serial.print("Program halt: low DO! Measured value: "); // halt program for 20 min to let DO value recover... This could be better used to generate an alarm output or activate air inflow
      Serial.println(lowDOValue);
      lcd.clear();
      lcd.print("low DO!");
      lcd.setCursor(0,1);
      lcd.print("measured: ");
      lcd.print(lowDOValue);
      delay(1200*1000UL);
      lowDO = false;
    }
    else if (lowDO == false){ 
      toggleRelay();                                            // operate relays to open solenoid valves
      endTime = millis();
      elapsed = endTime - startTime;                            // measure duration of loop
      if (elapsed > interval) {                                 // adjust measurement interval if loop takes longer than measurement interval
        interval = elapsed;
        lcd.clear();
        lcd.print("Short int.");
        lcd.setCursor(0, 1);
        lcd.print("New: ");
        lcd.print(interval/1000);
        lcd.print("sec");
      }
      else {
        delay(interval - elapsed);                               // adjust delay so that loop duration equals measurement interval
      }
    }
  }
}
