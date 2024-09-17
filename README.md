# __DecentralizedLV_Boards__ - CAN Bus Abstraction API

## Overview
DecentralizedLV-Boards is part of the [SolarPack project at NC State](https://www.solarpacknc.com/) which aims to create a practical solar-electric vehicle. This repository contains the code for the Decentralized Low Voltage (DecentralizedLV) API (aka Boards) which aims to abstract CAN Bus communication between the subcomponents in the car's 12-Volt accessory system. This README also details the layout of the DecentralizedLV system in the Car 2.0 12-Volt Accessory System.


## Decentralized Low Voltage System - A Cleaner Solution to Accessory System Wiring

The DecentralizedLV System aims to reduce clutter in the vehicle's accessory wiring using microcontrollers and a CAN communication bus. Instead of running many wires throughout the car for the individual components (such as headlights, high-beams, cooling pumps, fans, etc), the DecentralizedLV system has controllers on each corner of the vehicle. A controller is also placed near the driver instrumentation to read the inputs (turn signal stock, headlights, etc). The PCBs each have two wires for a 12-Volt supply, and two CAN communication wires. Each controller then has dedicated inputs and outputs for the accessories they power or the switches they read from the driver. Details on that in the [DecentralizedLV Subcomponents](#decentralizedlv-subcomponents) section.

<img width="521" alt="DecentralizedLV_Car1" src="https://github.com/user-attachments/assets/a600f599-255c-4059-88c8-b40bbafc6e67">


![DecentralizedLV_Car2](https://github.com/user-attachments/assets/a5840669-6fb7-40f6-90ff-c7b12c0a5bb1)


## DecentralizedLV Subcomponents
