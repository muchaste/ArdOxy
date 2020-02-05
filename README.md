# ArdOxy - Beta
An Arduino controlled system for long term automated oxygen control in fish tanks with FireStingO2 optical oxygen sensors.

The purpose of this readme is to make this system as barier-free and reproducible as possible. Below, I will outline the functioning principles of this system and explain basics and details, both of its hardware and software components. So, if you're new to pneumatics and coding, I hope that this documentation provides useful information for you. 

However, I would strongly recommend to anybody who is new to the field to start with a more simple project to learn the basics. You can find great, free tutorials and projects here:
* [Arduino community tutorials](https://www.arduino.cc/en/Tutorial/HomePage)
* [Tutorialspoint](https://www.tutorialspoint.com/arduino/)

## Project Status
This project is currently under development. The original sketch has been tested in an 8-week long term experiment, the new sketches are untested so far.

## Table of Contents
* [Background](#background)
* [Basic Setup](#basic-setup)
* [The Code](#the-code)
  * [The Preamble](#the-preamble)
  * [The Subfunctions](#the-subfunctions)
  * [Void Setup](#void-setup)
  * [The Main Loop](#the-main-loop)

## Background
Oxygen is a limited but essential resource for aquatic life. In many ecosystems, dissolved oxygen fluctuates and can reach critically low concentrations - a condition called hypoxia. Fish that have evolved under the pressure of aquatic hypoxia have developed many adaptations, ranging from behavioral strategies and morphology (-> gills!) to biochemical and physiological adjustments. These adaptations secure their survival under hypoxic conditions. For the research of these adaptations, it is advantageous if one can reproduce long term hypoxia (as it occurs naturally) in the lab.

A simple way to reproduce hypoxic conditions is to bubble nitrogen gas into water. The nitrogen displaces dissolved oxygen but remains otherwise inert - it does not react with the water. Hypoxic conditions, however, need to be closely monitored and controlled. Thus, periodic measurements and adjustments of oxygen levels are mandatory to create safe and reproducible conditions.

This project aims to create the basis for an open and reproducible oxygen control system on which interested researchers and afficionados can improve and develop their own systems. It is part of my PhD research on the behavior and physiology of weakly electric fish and I'm not a pro when it comes to pneumatic, kybernetics and electrotechnics (actually, I learned everything that I needed over the course of a couple of months). This means that
1. This system can still be improved
2. You can build it, even if you didn't go through an education in technical kybernetics

Here are the minimal requirements that I wanted to fulfill with this system:
* long-term measurement and logging of dissolved oxygen and temperature
* reproducible control of dissolved oxygen via nitrogen influx
* open and reproducible design
* affordability

## Basic Setup
*All components here are listed for reproduction purposes and not as advertisement!*

As this system is intended for long-term use, it relies on optical oxygen sensors. Optical sensors constitute the most expensive component - they are much more expensive than the relatively cheap electrochemical oxygen electrodes (such as [silver-platinum electrodes](https://en.wikipedia.org/wiki/Clark_electrode)). However, electrochemical sensors have a strong drift due to the deposition of salts on the anode and are thus not suited for long-term use without regularly being re-calibrated.
After a ton of research (there are many manufacturers of optical oxygen sensors out there), I decided for the FireStingO2 sensor with a 4-channel temperature extensions (TeX4) from PyroScience for the following reasons:

* 4 channel measurement on one device
* Serial communication via 3.3V-5V UART (-> can be hooked to a microcontroller!)
* usable as stand-alone sensor for other experiments
* supported by excellent and free software like [AquaResp](http://www.aquaresp.com/) and the [respR package](https://januarharianto.github.io/respR/index.html) for R
* relatively affordable - however the temperature extension TeX4 is quite expensive for what it gives you

This covers the measurement-side. Now for logging the values and actively controlling the oxygen we could have a PC (1000$) hooked to the sensors with a custom-written Matlab/LabView ($ license) and a lab-grade DAQ (400-1000$) to generate control outputs but that would mean we use a sledge-hammer to crack a nut and skyrocket the total cost of the system. By the way, it would violate my wish for it to be as open as possible.

Luckily, the FireStingO2 can be addressed via 3.3-5V serial communication - a language that most microcontrollers speak natively!
This means, instead of heavy equipment, we can use an arduino (or any clone) to "talk to" the sensor - *perfect!* The arduino is particularly suited for this because:
* it's cheap and open!
* it can be equipped with an LCD display, an SD card and a real time clock
* there's a great community and many libraries such as the PID library

Now what's still open is the pneumatic mechanism and control algorithm to bubble nitrogen into fish tanks. The obvious solution for this are solenoid valves - they're cheap and easy to control. The downside here is that simple solenoid valves work in a binary way - open and close. There's no ramping up the stream of gas that passes through, there's only bubble and stop. For my setup, I use these simple valves because you can get a decent one for about 30$. To control for the effect of bursts of bubbles (such as stress) and time change, I included 8 additional valves that bubble air into control tanks.

If you want a more sophisticated system, I suggest you check out *servo-assisted solenoid valves* (open/close dynamics depend on the pressure of the gas) or *proportional solenoid valves* (open/close dynamic can be controlled more finely) - they're more likely around 100-150$ a piece though.

My valves run on 24V, that means, they receive voltage, they open. This can be easily controlled via relays that are controlled by the arduino. Now, we only need the actual heart-piece of the whole control-system: a control algorithm. 
For this, I use one of the available and excellent [PID libraries](https://playground.arduino.cc/Code/PIDLibrary/) for the arduino. PID stands for proportional-integral-derivative and what it does, in short, is to keep track of control outputs and their effect in order to optimize the control output. 
An example helps to understand why we can't do without this: If you want to decrease the oxygen concentration from, say, 70% air saturation to 60% air saturation, a relatively short burst of nitrogen will do because at these concentrations, oxygen will only slowly diffuse back into the water. If you try to reach the same 10% decrease at lower air saturations, let's say from 20% to 10%, you'll need considerably more nitrogen gas. Why? Because now, there's a steep concentration gradient between the low oxygen concentration in the water and the comparably high concentration in the surrounding air. Thus, oxygen will constantly diffuse into the water and you have to actively drive it out. A PID control  "notices" that the same output of nitrogen leads to different outcomes and thus will increase the output at low air saturations. As we work with simple open/close valves, I let the PID controller calculate a time window for opening the valves. 

Okay, enough talk, here's a basic sketch of the most basic setup and its components:
![Overview](./images/simple_overview.png)
As mentioned above, the arduino sends to and receives from the sensor via its serial port. The relays are triggered via digital outputs and all is powered by separate power supplies. Also, as this remains untested at the moment, the temperature sensors are not included in this overview

Here's a list of the components of the last tested configuration for an 8-channel system:
* 1 Arduino MEGA 2560
* 1 Adafruit Datalogger Shield (ID1141)
* 1 Adafruit LCD displey (ID714)
* 2 FireStingO2 4 channel oxygen sensors + robust DO probes
* 2 Sainsmart 8-channel relays
* 16 24V burkert 6011 solenoid valves (8 for nitrogen and 8 for air)
* A lot of cable, gas tubes and push-in fittings

## The Code
You can find all the arduino sketches for this system here, in this repo. I commented the code as comprehensively as possible. However, some operating principles and core components of the code will be addressed here in a bit more detail.

*To edit and upload the sketches to an arduino, you need the [arduino IDE](https://www.arduino.cc/en/main/software)*.

For this documentation, I will refer to the original sketch that has been tested. The new versions are improvements of this original sketch and are similar in their basic structure. Let's have a look at it from top to bottom.

### The Preamble
In the preamble, the global variables are defined. "Global" are values and variables that all functions have access to. Here's an excerpt:
```javascript
...
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
...
```
The comments that are marked with a `//`make this code pretty self-explanatory. The preamble is structured into the general settings - here are the variables that you definitely have to adjust to your application (such as measurement channel number, oxygen thresholds etc.) - and further settings and variables that you only have to touch if you make more substantial changes to the system.

### The Subfunctions
Following (and still technically in the preamble), the individual subfunctions are defined that execute the different tasks (e.g. send measurement command to the sensor, print oxygen values on the display or create the control output for the valves). It's not necessary to create subfunctions, I could write all the code just in the main `loop` (see below). However, it makes everything clearer and easier to understand if large chunks of code are "hidden" in these subfunctions and we only have to call them by their name later. Let's have a look at the top part with the first subfunction:
```javascript
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
```
Let's go through this step by step:
* `toggleMeasurement` is the name of the subfunction - a function to toggle the oxygen measurement by sending measurement commands to the oxygen sensor via the arduino's serial port. `void` declares it as a function that does not return any value (pretty symbolic, eh?). It is followed by `(char command[6])` which means that this function uses a string with a length of 6 characters as input argument when it's called - e.g. `toggleMeasurement("MSR 1")`. Note that the string "MSR 1" is only 5 characters long - the last space is always reserved for the string terminator (/0) which marks the end of your input string. You might wonder how to choose the commands - well, every device has its own language. The FireStingO2 understands a set of different string commands that will trigger measurement, readout of values or change the settings. Usually, you can get a documentation of the communication protocol - kind of a vocabulary list of the sensor - from the manufacturer.
* As you can see, the rest of the subfunction consists of two pretty similar chunks of code - the only difference is the serial port that is used. These two block of code are separated by an `if` statement: if sensor one is active, the command is sent through serial port one and so on. This is necessary because every sensor is connected to the arduino by a separate serial port. *Note: in the newer sketches, the serial port is dynamically changed and we only need one chunk of code. However, this makes the principle clearer.*
* Next comes one of the core functions of this code: `Serial.write`. It sends whatever you pass on to it through the serial port. As you can see, we pass it the input string `command` that we define when we call the subfunction (e.g. "MSR 1"). In the next line, the same function is used to send `\r`, the code for "carriage return" (imagine a typewriter - "ding!") that symbols the sensor that the command has been sent completely and can now be executed by the sensor.
* Finally, `Serial.flush()` halts the arduino until all the data (the command) has left the serial buffer. This is an important part as it makes sure that the sending of the command is executed completely before any other part of the program runs

### Void Setup
Next comes `void Setup()`. This section runs once when the arduino boots. Everything that the system needs to boot is defined here - the logfile is created and the serial ports and pins are activated. (Code will come soon)

### The Main Loop
Finally, the main `loop`. The arduino will run everything that you write here over and over again - hence its name. Here are all the subfunctions called that are needed for measuring and controlling oxygen. (Code will come soon).

