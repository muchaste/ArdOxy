# ArdOxy - Beta
An Arduino controlled system for long term automated oxygen control in fish tanks with FireStingO2 optical oxygen sensors.
*This project is currently under development - the original sketch has been tested in a 8-week long-term test (see below).*

## Table of Contents
* [Background](#background)
* [Basic Setup](#basic-setup)
* [The Code](#the-code)

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

Now what's still open is the pneumatic mechanism to bubble nitrogen into fish tanks. The obvious solution for this are solenoid valves - they're cheap and easy to control. The downside here is that simple solenoid valves work in a binary way - open and close. There's no ramping up the stream of gas that passes through, there's only bubble and stop. For my setup, I use these simple valves because you can get a decent one for about 30$. To control for the effect of bursts of bubbles (such as stress) and time change, I included 8 additional valves that bubble air into control tanks.
If you want a more sophisticated system, I suggest you check out *servo-assisted solenoid valves* (open/close dynamics depend on the pressure of the gas) or *proportional solenoid valves* (opening and closing dynamic can be controlled) - they're more likely around 100-150$ a piece though.
My valves run on 24V, that means, they get voltage, they open. This can be easily controlled via relais that are controlled by the arduino - done!

Okay, enough talk, here's a basic sketch of the most basic setup and its components:
![Overview](./images/simple_overview.png)

Here's a list of the components of the last tested configuration for an 8-channel system:
* 1 Arduino MEGA 2560
* 1 Adafruit Datalogger Shield (ID1141)
* 1 Adafruit LCD displey (ID714)
* 2 FireStingO2 4 channel oxygen sensors + robust DO probes
* 2 Sainsmart 8-channel relays
* 16 24V burkert 6011 solenoid valves (8 for nitrogen and 8 for air)
* A lot of cable, gas tubes and push-in fittings

## The Code
TBA
