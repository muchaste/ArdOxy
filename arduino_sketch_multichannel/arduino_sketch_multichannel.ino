//#######################################################################################
//###                 ArdOxy - Arduino based oxygen control system                    ###
//### Core hardware components:                                                       ###
//### Arduino MEGA 2560 (1x), Adafruit Data logging shield (ID 1141),                 ###
//### Adafruit LCD shield (ID 714), SainSmart 8 channel relay module (101-70-102),    ###
//### PyroScience FirestingO2 (2x), solenoid valves (8x)                              ###
//### -------------- Read the documentation before using this sketch!---------------- ###
//### Adapt this sketch to fit your application and carefully test the system before  ###
//### using it.                                                                       ###
//### Low oxygen is potentially harmful, so use this sketch carefully and only under  ###
//### monitored conditions. I take no liability for damages caused by misuse of this  ###
//### sketch.                         ~Stefan Mucha, 2019                             ###
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
const int s2ChannelNumber = 2;                                                          // number of used channels on sensor 2
const int channelNumber = s1ChannelNumber + s2ChannelNumber;                            // total number of measurement channels
long int samples = 3;                                                                   // number of measurements that are averaged to one air saturation value 
                                                                                        // to reduce sensor fluctuation (oversampling)
char fishID[8][6] = {"B4", "C3", "B1", "E7", "E3", "ID6", "ID7", "ID8"};           // IDs assigned to the channels in the order of the channelArray
long interval = 30 * 1000UL;                                                            // measurement and control interval in second
double airSatThreshold[8] = {15.0, 15.0, 15.0, 70.0, 30.0, 60.0, 100.0, 100.0};    // air saturation threshold including first decimal                            
const int chipSelect = 10;                                                              // chip pin for SD card (UNO: 4; MEGA: 53, Adafruit shield: 10)
double securityThreshold = 12.0;                                                        // threshold for low oxygen that causes the program to halt
int channelArray[8] = {1, 2, 3, 4, 1, 2, 3, 4};                                         // input channels on firesting devices 1 and 2 in that order



//#######################################################################################
//###                                Relay settings                                   ###
//### The arduino operates solenoid valves through a relay module to bubble N2 into   ###
//### the fish tanks. It calculates a time interval using the PIDlibrary (by Brett    ###
//### Beauregard). TEST THE SETTINGS FOR PID CONTROL BEFORE YOU USE THIS SYSTEM WITH  ###
//### FISH!! If you're not sure, replace the PID control with a simpler solution      ###
//### (e.g. bubble N2 for 10 sec if there's a large oxygen difference etc.).          ###
//#######################################################################################

int relayPin[2][8] = {                                          // pins for relay operation ***ADAPT THIS TO FIT YOUR WIRING***
  {22, 24, 26, 28, 30, 32, 34, 36},
  {23, 25, 27, 29, 31, 33, 35, 37}
};     
double Kp[8] = {10, 10, 10, 10, 10, 10, 10, 10};    // coefficient for proportional control
double Ki[8] = {1, 1, 1, 1, 0, 0, 0, 0};            // coefficient for integrative control
double Kd[8] = {1, 1, 1, 1, 0, 0, 0, 0};            // coefficient for differential control
long int WindowSize = 50;                                       // The PID will calculate an output between 0 and 50.
                                                                // This will be multiplied by 200 to ensure a minimum opening time of 200 msec. 
                                                                // E.g. output = 1 -> opening time 200 msec; output 50 -> opening time 10,000 msec

//#######################################################################################
//###                           Requisite variables                                   ###
//### In the following, necessary variables are defined. If you don't change too much ###
//### of this code, you will not need to change a lot here but you will need to read  ###
//### through it at least to understand how the functions works.                      ###
//#######################################################################################

//# Switches and logical operators #
boolean lowDO = false;                        // boolean for low oxygen threshold
int activeSensor = 1;                         // index to switch between sensors (if s2ChannelNumber = 0 it stays at 1)
int activeChannel = 1;                        // index to switch bewteen channels
boolean newData = false;                      // true if serial data was received from sensors
boolean emptyBuffer = false;                  // true if buffer for serial data is empty

//# Measurement timing #
unsigned long startTime;                      // time taken at start of loop
unsigned long endTime;                        // time taken at end of loop
long elapsed;                                 // elapsed time between end- and start-time

//# Logging and SD card #
RTC_PCF8523 RTC;
File logfile;                                 // initializes the logfile
char filename[16];                            // array for filename of .csv file
int fn = 0;                                   // filename index for .csv file
int n = 0;                                    // row index for logging

//# Oxygen optode #
char toggleMeasString[6] = "SEQ 1";           // measurement commmand that is sent to sensor during void toggleMeasurement(). SEQ triggers a sequence of measurements 
                                              // (humidity, temperature, pressure, air saturation). This way, the air saturation values will be compensated automatically
                                              // for fluctuations in humidity, temperature and pressure.
char toggleReadString[10] = "REA 1 3 4";      // template for read command that is sent to sensor during void toggleRead()
const byte numChars = 80;                     // array length for incoming serial data (longer than needed)
char receivedChars[numChars];                 // array to store the incoming serial data
int len = 0;                                  // length of actually received serial data
char airSatStr[20];                           // array to store only air saturation readings from receivedChars (longer than needed)
long airSatReading = 0;                       // variable for air saturation readings as long integer parsed from string
long airSatSum = 0;                           // summing variable for air saturations (see sample variable)
double airSatFloat[8];                        // array for floating point value of air saturation for logging, display etc.
char airSatDisplay;                           // string template for serial display of measurements
double lowDOValue;                            // variable for critically low DO values in case they appear

//# Relay #
double relayArray[4][8];        // array with relay pin and assigned output values
double Output[8];               // holds output that was calculated by PID library
//Set up all PIDs - there's probably a way to set up only those that are needed but I can only think of a long chain of if-and conditions
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
int airSatLCD = 0;


//#######################################################################################
//###                           Requisite Functions                                   ###
//###               These functions are executed in the main() loop.                  ###
//### The functions for receiving serial data, parsing it and clearing the buffer are ###
//### modified from a great post on the Arduino Forum by user Robin2 about the basics ###
//### of serial communication (http://forum.arduino.cc/index.php?topic=396450).       ###
//#######################################################################################

//# Send the commands to toggle measurement and readout DO values to the FireStingO2 via Serial #
void toggleMeasurement(char command[6]) {
  if (activeSensor == 1) {
    Serial1.write(command);
    Serial1.write('\r');
    emptyBuffer = false;
  }
  else if (activeSensor == 2) {
    Serial2.write(command);
    Serial2.write('\r');
    emptyBuffer = false;
  }
}

void toggleRead(char command[10]) {
  if (activeSensor == 1) {
    Serial1.write(command);
    Serial1.write('\r');
  }
  else if (activeSensor == 2) {
    Serial2.write(command);
    Serial2.write('\r');
  }
}

//# Receive serial data (modified from http://forum.arduino.cc/index.php?topic=396450) #
void recvWithEndMarker() {                                                              // receives serial data and stores it in array until endmarker is received
  static byte ndx = 0;                                                                  // index for storing in the array
  char endMarker = '\r';                                                                // declare the character that marks the end of a serial transmission
  char rc;                                                                              // temporary variable to hold the last received character
  if (activeSensor == 1) {
    while (Serial1.available() > 0 && newData == false && emptyBuffer == true) {        // only read serial data if the buffer was emptied before and it's new data
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
        newData = true;
      }
    }
  }
  else if (activeSensor == 2) {                                                         // same as above for the second sensor
    while (Serial2.available() > 0 && newData == false && emptyBuffer == true) {
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
      }
    }
  }
}

//# Clear the serial buffer (modified from http://forum.arduino.cc/index.php?topic=396450) #
void clearBuffer() {                                                                    // similar to recvWithEndMarker(), clears serial buffer from echoes
  static byte ndx = 0;
  char endMarker = '\r';
  char rc;
  if (activeSensor == 1) {
    while (Serial1.available() > 0 && emptyBuffer == false) {
      rc = Serial1.read();
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0';  // terminate the string
        ndx = 0;
        emptyBuffer = true;
        newData = false;
      }
    }
  }
  else if (activeSensor == 2) {
    while (Serial2.available() > 0 && emptyBuffer == false) {
      rc = Serial2.read();
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0';  // terminate the string
        ndx = 0;
        emptyBuffer = true;
        newData = false;
      }
    }
  }
}

//# Extract air saturation values from received serial data #
int extractAirSat() {            
  len = strlen(receivedChars)+1;                  // get length of received string + termination \0
  if (len == 0) {                                 // check if there was an answer from the sensor
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Strlen = 0 :(");
    lcd.setCursor(0, 1);
    lcd.print("Sensor connected?");
    while (1);  
  }
  len = len - 10;                                 // subtract to length of DO measurement digits - I determined this by testing it, there might be a smarter way to do this
  for (int i = 0; i < len; i++) {                 // store the last digits of string in new object airSatStr, starting at and including position 10. This is necessary as
    airSatStr[i] = receivedChars[(10 + i)];       // the sensor echoes the readout command, e.g. "REA 1 3 4" before the air saturation values
  }
  airSatReading = atol(airSatStr);                // parse the character to integers - the air saturation values are given as [% air saturation x 1000]
  return airSatReading;
}

//# Send air saturation readings to serial monitor and LCD (sets newData to false) #
void showNewData() {
  if (newData == true && emptyBuffer == true) {
    lcd.clear();
    lcd.setCursor(0, 0);
    for (int i = 0; i < (channelNumber); i++) {
      if (i == 4) {                                     
        lcd.setCursor(0, 1);                          // break line on LCD display when the 5th DO value is reached
      }
      Serial.print("Floating point values: ");
      Serial.print(airSatFloat[i]);
      Serial.print("; ");
      airSatLCD = int(lround(airSatFloat[i]));
      lcd.print(airSatLCD);
      lcd.print(" ");
    }
    Serial.println();
    newData = false;
  }
}

//# Check if air saturations are above the security threshold #
void securityCheck() {
  for (int i = 0; i < channelNumber; i++){
    if (airSatFloat[i] < securityThreshold){
      lowDO = true;
      lowDOValue = airSatFloat[i];
    }
    else {
      lowDO = false;
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
  
  long int temp[4];                                     // temporary array to sort all channels with the smallest based on the computed output (lowest output first). 
                                                        // This is necessary as the valves are kept open using the delay()-function which halts all activity. 
                                                        // With 8 channels, opening one valve after the other is too time consuming. With the opening times sorted, all valves 
                                                        // can be opened at the same time and then closed one after another.
  for (int i = 0; i < channelNumber - 1; i++) {         // loop to sort array with lowest difference first
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

  for (int i = 0; i < channelNumber; i++) {             // print input and output to serial monitor - this is handy to control, how the PID is behaving
    Serial.print("Relay Pin: ");
    Serial.print(relayArray[0][i]);
    Serial.print("Input: ");
    Serial.print(relayArray[3][i]);
    Serial.print("; Output: ");
    Serial.println(relayArray[2][i]);
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
    for (int i = 0; i < channelNumber; i++) {     // print air saturation measurements for each channel
      logfile.print(airSatFloat[i]);
      logfile.print(";");
    }
    logfile.println();
    logfile.flush();                                  // save data to logfile
  }
}

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
  delay(500);

//# Declare output pins for relay operation #
  lcd.clear();
  lcd.print("Relay pins..");
  for (int i = 0; i < channelNumber; i++) {       // declare relay pins as output pins and write HIGH to close magnetic valves
    pinMode(relayPin[0][i], OUTPUT);
    pinMode(relayPin[1][i], OUTPUT);
    digitalWrite(relayPin[0][i], HIGH);
    digitalWrite(relayPin[1][i], HIGH);
  }
  delay(500);

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
  RTC.adjust(DateTime(F(__DATE__), F(__TIME__))); // set rtc to the time this sketch was compiled
  delay(500);
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
    logfile.print("Date:;");                                // print header information: date, time, air saturation threshold, channels, fish IDs etc
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
    logfile.println(channelNumber);
    logfile.print("Air sat threshold [% air saturation]:;");
    for (int i = 0; i < (channelNumber); i++) {
      logfile.print(airSatThreshold[i]);
      logfile.print(";");
    }
    logfile.println();
    logfile.print("Fish ID:;");
    for (int i = 0; i < (channelNumber - 1); i++) {
      logfile.print(fishID[i]);
      logfile.print(";");
    }
    logfile.println(fishID[channelNumber - 1]);
    logfile.print("Channel:;");
    for (int i = 0; i < (channelNumber - 1); i++) {
      logfile.print(channelArray[i]);
      logfile.print(";");
    }
    logfile.println(channelArray[channelNumber - 1]);
    logfile.println(";");
    logfile.print("Measurement;Date;Time;");                  // header row for measurements: Measurement, Date, Time, FishID1, FishID2,...
    for (int i = 0; i < (channelNumber); i++) {
      logfile.print(fishID[i]);
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
  for (int i = 0; i < channelNumber; i++) {                   // close all valves to make sure
    digitalWrite(relayPin[0][i], HIGH);
    digitalWrite(relayPin[1][i], HIGH);
  }
  for (int i = 0; i < channelNumber; i++) {                   // loop that iterates through every channel
    airSatSum = 0;                                            // reset summing variable for measurements
    airSatReading = 0;                                        // reset air saturation holder variable
    if (i < s1ChannelNumber) {                                // switch sensor based on measurement channel
      activeSensor = 1;
    }
    else if (i == s1ChannelNumber) {
      activeSensor = 2;
    }  
    activeChannel = channelArray[i];                          // declare channel to be measured for serial commands
    sprintf(toggleMeasString, "SEQ %d", activeChannel);       // insert channel in measurement command
    sprintf(toggleReadString, "REA %d 3 4", activeChannel);   // insert channel in readout command
    for (int j = 0; j < samples; j++) {                       // oversampling loop per channel
      toggleMeasurement(toggleMeasString);
      delay(500);                                             // DON'T SHORTEN THIS DELAY! It's the necessary time for the sensor to measure
      clearBuffer();
      delay(100);
      toggleRead(toggleReadString);
      delay(100);
      recvWithEndMarker();
      delay(100);
      extractAirSat();
      delay(100);
      airSatSum = airSatSum + airSatReading;                  // sum up air saturation readings for consecutive samples
      Serial.print("airSatStr: ");                            // print the received data, its converted values etc. for control
      Serial.println(airSatStr);
      Serial.print("airSatReading: ");
      Serial.println(airSatReading);
      Serial.print("receivedChars: ");
      Serial.println(receivedChars);
    }
    airSatFloat[i] = airSatSum / (samples * 1000.00);         // create floating point number for logging, display, etc.
  }
  showNewData();                                              // send measurement to serial monitor
  delay(100);
  writeToSD();                                                // log to SD card
  delay(100);
  securityCheck();                                            // check if low DO threshold is crossed
  if (lowDO == false){ 
    toggleRelay();                                            // operate relay
    endTime = millis();
    elapsed = endTime - startTime;                            // measure duration of loop
    if (elapsed <= interval) {
      delay(interval - elapsed);                              // adjust delay so that loop duration equals measurement interval
    }
    else {                                                    // halt if loop takes longer than measurement interval
      interval = elapsed;
      Serial.print("Initial measurement interval too short, new interval [s]: ");
      Serial.println(interval/1000);
      lcd.clear();
      lcd.print("Short int.");
      lcd.setCursor(0, 1);
      lcd.print("New: ");
      lcd.print(interval/1000);
      lcd.print("sec");
    }
  }
  else if (lowDO == true){
    Serial.print("Program halt: low DO! Measured value: ");
    Serial.println(lowDOValue);
    lcd.clear();
    lcd.print("low DO!");
    lcd.setCursor(0,1);
    lcd.print("measured: ");
    lcd.print(lowDOValue);
    while (1);
  }
}
