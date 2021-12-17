
/*
  Ardoxy example - measure and control 4 channels via solenoids 
  
  Trigger a measurement sequence (DO, temperature, air pressure) and read out the results. 
  Display on LCD and store on SD card.
  Set desired DO level by opening a solenoid valve. 
  Opening time is calculated with PID library.
  
  Oxygen sensor is calbrated using the Pyro Oxygen Logger Software.
  
  The circuit:
  - Arduino Uno
  - FireStingO2 - 7 pin connector:
    *Pin 1 connected to Arduino GND
    *Pin 2 connected to Arduino 5V 
    *Pin 4 connected to Arduino RX (here: 10)
    *Pin 5 connected to Arduino TX (here: 9)
  - Solenoid valve on relay module, connected to digital pins on Arduino (here: 2-5)

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
#include "SD.h"
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#define WHITE 0x7                                     // LCD color code


//#######################################################################################
//###                              General settings                                   ###
//### This program is intended to work with 4 channels for measurement and control.   ###
//### Thus most arrays have a length of 4. Adapt the following lines to your use.     ###
//#######################################################################################

// Set experimental conditions
const int channelNumber = 4;                                                                  // total number of measurement channels
long int samples = 1;                                                                         // number of measurements that are averaged to one air saturation value 
                                                                                              // to reduce sensor fluctuation (oversampling)
char tankID[channelNumber][6] = {"A", "B", "C", "D"};                                         // IDs assigned to the channels in the order of the channelArray
char tempID[6] = {"B"};                                                                       // tanks where the temperature sensors are placed (1 per sensor)
long sampleInterval = 30 * 1000UL;                                                            // measurement and control interval in second
double airSatThreshold[channelNumber] = {100.0, 100.0, 100.0, 15.0};                          // air saturation threshold including first decimal                            
double lowDOThreshold = 7.0;                                                                  // threshold for low oxygen that causes the program to do something
int channelArray[channelNumber] = {1, 2, 3, 4};                                               // measurement channels from firesting devices 1 and 2 in that order

// Define pins
const int RX = 10;                                    // RX and TX pins for serial communication with FireStingO2
const int TX = 9;                                     // RX and TX pins for serial communication with FireStingO2
int relayPin[channelNumber] = {5, 6, 7, 8};                     // pins for relay operation ***ADAPT THIS TO FIT YOUR WIRING***
const int chipSelect = 10;                                                                    // chip pin for SD card (UNO: 4; MEGA: 53, Adafruit shield: 10)


//#######################################################################################
//###                                PID settings                                     ###
//### The arduino operates solenoid valves through a relay module to bubble N2 into   ###
//### the tanks. It calculates a time interval using the PIDlibrary (by Brett         ###
//### Beauregard). TEST THE SETTINGS FOR PID CONTROL BEFORE YOU USE THIS SYSTEM!!     ###
//### If you're not sure, replace the PID control with a simpler solution             ###
//### (e.g. bubble N2 for 10 sec if there's a large oxygen difference etc.).          ###
//### To control for stress due to bubbling, compressed air is bubbled into control   ###
//### tanks.                                                                          ###
//#######################################################################################

double Kp[channelNumber] = {10, 10, 10, 10};                    // coefficient for proportional control
double Ki[channelNumber] = {1, 1, 1, 1};                        // coefficient for integrative control
double Kd[channelNumber] = {1, 1, 1, 1};                        // coefficient for differential control
const int windowSize = 75;                                       // The PID will calculate an output between 0 and 75.
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
int activeChannel = 1;                        // measurement channel
int check;                                    // numerical indicator of succesful measurement (1: success, 0: no connection, 9: mismatch)

//# Measurement timing #
unsigned long loopStart, elapsed;             // ms timestamp of beginning and end of measurement loop
int curday, lastday;                          // int of current day and last day (date) - to detect change and create a new logfile every day

//# Logging and SD card #
RTC_PCF8523 RTC;                              // real time clock
File logfile;                                 // initializes the logfile
char filename[16];                            // array for filename of .csv file
byte n = 0;                                   // row index for .csv file

//# Oxygen optode #
char seqMeasCom[7] = "SEQ 1\r";               // measurement commmand that is sent to sensor during void toggleMeasurement(). SEQ triggers a sequence of measurements 
                                              // (humidity, temperature, pressure, air saturation). This way, the air saturation values will be compensated automatically
                                              // for fluctuations in humidity, temperature and pressure.
char DOReadCom[11] = "REA 1 3 4\r";           // template for DO-read command that is sent to sensor during void toggleRead() (length = 10 because of /0 string terminator)
char tempReadCom[11] = "REA 1 3 5\r";         // temperature read command
long DOInt, tempInt;                          // for measurement result
long DOSum;                                   // summing variable for air saturations in case oversampling is used (see sample variable)
double DOFloat[channelNumber], tempFloat;     // measurement result as floating point number
double lowDOValue;                            // variable for low DO values that are below the critical threshold defined above
char lowDOTank[6];                            // array for tank name in which low DO has been measured
SoftwareSerial FSSerial(RX,TX);               // 
Ardoxy ardoxy(&FSSerial);                     //

//# Relay operation #
double relayArray[3][channelNumber];          // array with relay pin and assigned output values
double Output[channelNumber];                 // holds output that was calculated by PID library

//Hardcoded PID setup - there's probably a way to set up only those that are needed but I can only think of a long chain of if-and conditions
PID relay1PID(&DOFloat[0], &Output[0], &airSatThreshold[0], Kp[0], Ki[0], Kd[0], REVERSE);    
PID relay2PID(&DOFloat[1], &Output[1], &airSatThreshold[1], Kp[1], Ki[1], Kd[1], REVERSE);
PID relay3PID(&DOFloat[2], &Output[2], &airSatThreshold[2], Kp[2], Ki[2], Kd[2], REVERSE);
PID relay4PID(&DOFloat[3], &Output[3], &airSatThreshold[3], Kp[3], Ki[3], Kd[3], REVERSE);

//# LCD Display #
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
int airSatLCD = 0;                            // integer to display rounded values (due to space constraints on the LCD)


//#######################################################################################
//###                           Requisite Functions                                   ###
//###               These functions are executed in the main() loop.                  ###
//### The functions for receiving serial data, parsing it and clearing the buffer are ###
//### modified from the excellent post on the Arduino Forum by user Robin2 about the  ###
//### basics of serial communication (http://forum.arduino.cc/index.php?topic=396450).###
//#######################################################################################

//# Send air saturation readings to serial monitor and LCD #
void showNewData() {
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
  Serial.print(tempID);
  Serial.print(": ");
  Serial.print(tempFloat);
  Serial.println("°C ");

  for (int k = 0; k < (channelNumber); k++) {
    if (k == 4) {                                     
      lcd.setCursor(0, 1);                          // break line on LCD display when the 5th DO value is reached
    }
    airSatLCD = int(lround(DOFloat[k]));
    lcd.print(airSatLCD);
    lcd.print(" ");
    Serial.print(tankID[k]);
    Serial.print(": ");
    Serial.print(DOFloat[k]);
    Serial.println("% air saturation");
  }
}


//# Check if air saturations are below the lowDO threshold #
void DOCheck() {
  for (int k = 0; k < channelNumber; k++){
    if (DOFloat[k] < lowDOThreshold){
      lowDO = true;
      lowDOValue = DOFloat[k];
      strcpy(lowDOTank, tankID[k]);
    }
  }
}

//# Toggle relay based on measured airSat values #
void toggleRelay() {
  relay1PID.Compute();                                  // compute the output (output * 200 = opening time) based on the input (air saturation) and threshold
  relay2PID.Compute();
  relay3PID.Compute();
  relay4PID.Compute();
  
  for (int k = 0; k < channelNumber; k++) {
    relayArray[0][k] = relayPin[k];
    relayArray[1][k] = double(int(Output[k])*200.00);   // PID computes an output between 0 and 50, the multiplicator makes sure that the relay operation time is at least 200ms
    relayArray[2][k] = DOFloat[k];
  }
  double temp[4];                                       // temporary array to sort all channels with the smallest based on the computed output (lowest output first). 
                                                        // This is necessary as the valves are kept open using the delay()-function which halts all activity. 
                                                        // With 8 channels, opening one valve after the other is too time consuming. With the opening times sorted, all valves 
                                                        // can be opened at the same time and then closed one after another.
  for (int k = 0; k < channelNumber - 1; k++) {         // sort array with lowest difference first
    for (int m = k + 1; m < channelNumber; m++) {
      if (relayArray[1][m] < relayArray[1][k]) {
        temp[0] = relayArray[0][k];                     // store higer value + relayPin in temp array
        temp[1] = relayArray[1][k];
        temp[2] = relayArray[2][k];
        relayArray[0][k] = relayArray[0][m];            // move lower value to position i
        relayArray[1][k] = relayArray[1][m];
        relayArray[2][k] = relayArray[2][m];
        relayArray[0][m] = temp[0];                     // insert higher value at position j
        relayArray[1][m] = temp[1];                     // now the values at i and j have switched places in the relayArray
        relayArray[2][m] = temp[2];
      }
    }
  }
  for (int k = 0; k < channelNumber; k++) {
    if (relayArray[1][k] > 0){                              // skip the channels that don't have to be operated (output = 0.00)
      for (int m = k; m < channelNumber; m++) {             // open all other valves
        digitalWrite(relayArray[0][m], LOW);
      }
      if (k == 0) {                                         // close the valve with the lowest output first
        delay(relayArray[1][k]);
        digitalWrite(relayArray[0][k], HIGH);
      }
      else {
        delay(relayArray[1][k] - relayArray[1][k - 1]);     // keep the other valves open based on the difference of the output value
        digitalWrite(relayArray[0][k], HIGH);               // close the valve
      }
    }
  }
}

//# Create logfile on SD card (needs global variable "filename")
void createLogfile(){
  DateTime now;
  now = RTC.now();                                  // fetch time and date from RTC
  
  snprintf(filename, sizeof(filename), "%d_%d_%d_%d_%d.csv", now.year(), now.month(), now.day(), now.hour(), now.minute()); // choose filename for logfile on SD that does not exist yet, includes a three-digit sequence number in the file name
//    while (SD.exists(filename)) {
//      fn++;
//      snprintf(filename, sizeof(filename), "dolog%03d.csv", fn);
//    }
    //logfile = SD.open(filename, FILE_READ);
    //logfile.close();                                          // now filename[] contains the name of a file that doesn't exist

  logfile = SD.open(filename, FILE_WRITE);
    
//# Create logfile #
  //logfile = SD.open(filename, FILE_WRITE);                  // create logfile and write header information
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
    logfile.print(sampleInterval / 1000);
    logfile.print(";Active channels:;");
    logfile.print(channelNumber);
    logfile.print(";Temp Sensor:;");
    logfile.print(tempID);
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
    logfile.print(tempID);
    logfile.print(";");
    for (int i = 0; i < (channelNumber); i++) {
      logfile.print(tankID[i]);
      logfile.print(";");
    }
    logfile.println();

    // Print to Serial monitor
    Serial.print("Logfile created: ");
    Serial.println(filename);

    // Print to LCD
    lcd.setCursor(0,1);
    lcd.print(filename);
  }
  else if (!logfile) {                                        // check if logfile was created
    Serial.println("error: couldn't create logfile");
    lcd.setCursor(0, 1);
    lcd.print(".csv failed");
    while (1);                                                // do nothing
  }
}

//# Log dissolved oxygen measurements to SD card #
void writeToSD() {
  if (!SD.exists(filename)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("error!");
    lcd.setCursor(0, 1);
    lcd.print("no SD");
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
    logfile.print(tempFloat);
    logfile.print(";");
    for (int k = 0; k < channelNumber; k++) {         // print air saturation measurements for each channel
      logfile.print(DOFloat[k]);
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
  ardoxy.begin(19200);
    
//# Set up one PID per channel #
  relay1PID.SetMode(AUTOMATIC);
  relay1PID.SetSampleTime(sampleInterval);
  relay1PID.SetOutputLimits(0, windowSize);
  relay2PID.SetMode(AUTOMATIC);
  relay2PID.SetSampleTime(sampleInterval);
  relay2PID.SetOutputLimits(0, windowSize);
  relay3PID.SetMode(AUTOMATIC);
  relay3PID.SetSampleTime(sampleInterval);
  relay3PID.SetOutputLimits(0, windowSize);
  relay4PID.SetMode(AUTOMATIC);
  relay4PID.SetSampleTime(sampleInterval);
  relay4PID.SetOutputLimits(0, windowSize);
  
//# Start LCD display, clear serial buffer #
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DO ctrl booting.");
  delay(100);

//# Declare output pins for relay operation #
  lcd.clear();
  lcd.print("Relay pins..");
  for (int i = 0; i < channelNumber; i++) {       // declare relay pins as output pins and write HIGH to close magnetic valves
    pinMode(relayPin[i], OUTPUT);
    digitalWrite(relayPin[i], HIGH);
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

//# Display time on LCD display #
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
  delay(2000);                                    // long delay to check the date on the display

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

//# Create a new logfile #
  lcd.clear();
  lcd.print("Create .csv...");
  
  createLogfile();
  delay(500);

//# Display control settings 
  Serial.print("Start of measurement cycles. Measurement interval set at ");
  Serial.print(sampleInterval / 1000);
  Serial.println(" seconds.");
  lcd.clear();
  lcd.print("Ready");
  lcd.setCursor(0, 1);
  lcd.print("Interval: ");
  lcd.print(sampleInterval / 1000);
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

  // Set lastday for saving every day
  lastday = now.day();
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
  loopStart = millis();                                       // start timer of loop
  DateTime now;
  now = RTC.now();  
  curday = now.day();
  if (curday != lastday){                                     // system restarts every day -> creates a new logfile for every day
    logfile.close();
    delay(100);
    createLogfile();
    lastday = curday;
  }
  
  for (int i = 0; i < channelNumber; i++) {                   // loop that iterates through every channel
    DOSum = 0;                                                // reset summing variable for measurements
    DOInt = 0;                                             // reset value variable

    activeChannel = channelArray[i];                          // declare channel to be measured for serial commands
    sprintf(seqMeasCom, "SEQ %d\r", activeChannel);       // insert channel in measurement command
    sprintf(DOReadCom, "REA %d 3 4\r", activeChannel);       // insert channel in readout command
    for (int j = 0; j < samples; j++) {                       // oversampling loop 
      check = ardoxy.measure(seqMeasCom);
      if (check == 1){
        DOInt = ardoxy.readout(DOReadCom);
        if (i == 0){
          tempInt = ardoxy.readout(DOReadCom);
          tempFloat = tempInt / 1000.00;
        }
      } else {
        DOInt = 0;
      }
      DOSum = DOSum + DOInt;                     // sum up air saturation readings for consecutive samples
    }
    DOFloat[i] = DOSum / (samples * 1000.00);         // create floating point number for logging, display, etc. Results in 0 if there's a communication error
  }
  showNewData();                                            // display measurement on LCD
  delay(100);
  writeToSD();                                              // log to SD card
  delay(100);
  DOCheck();                                                // check if low DO threshold is crossed
  if (lowDO == true){
    Serial.print("Program halt: low DO! Measured value: "); // halt program for 20 min to let DO value recover... This could be better used to generate an alarm output or activate air inflow
    Serial.println(lowDOValue);
    lcd.clear();
    lcd.print("low DO at ");
    lcd.print(lowDOTank);
    lcd.setCursor(0,1);
    lcd.print("measured: ");
    lcd.print(lowDOValue);
    delay(1200*1000UL);
    lowDO = false;
  }
  else if (lowDO == false){ 
    toggleRelay();                                            // operate relays to open solenoid valves
    elapsed = millis() - loopStart;                            // measure duration of loop
    if (elapsed > sampleInterval) {                                 // adjust measurement interval if loop takes longer than measurement interval
      sampleInterval = elapsed;
      lcd.clear();
      lcd.print("Short int.");
      lcd.setCursor(0, 1);
      lcd.print("New: ");
      lcd.print(sampleInterval/1000);
      lcd.print("sec");
    }
    else {
      delay(sampleInterval - elapsed);                               // adjust delay so that loop duration equals measurement interval
    }
  }
}
