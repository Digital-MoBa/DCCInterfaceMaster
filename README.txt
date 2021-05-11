DDCCInterfaceMaster (C) Philipp Gahtow
base on CmdrArduino

This library create a DCC Signal with RailCom (optional). It can handle two DCC outputs, one with Power feature and 
one permanent for driving LocoNet Railsync or S88N Raildata line.

===========

modified by Philipp Gahtow 2015-201 digitalmoba@arcor.de
* - add a store for active loco, so you can request the actual state
* - add a store for BasicAccessory states
* - add a repeat queue for Speed and Function packets
* - add Function support F13-F20 and F21-F28
* - add CV POM Messages
* - add BasicAccessory increment 4x (Intellibox - ROCO)
* - add request for state of Loco funktion F0 - F28
* - support DCC generation with Timer1 or Timer2
* - add notify of BasicAccessory even when power is off
* - change praeambel to 16 Bit for Railcom support
* - add Railcom hardware support 
* - optimize Railcom signal timing
* - fix bug on ESP32 (https://github.com/crosstool-ng/crosstool-ng/issues/1330)
===========


To install, see the general instructions for Arduino library installation here:
http://arduino.cc/en/Guide/Environment#libraries
