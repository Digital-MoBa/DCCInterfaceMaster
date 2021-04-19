DCCInterfaceMaster (C) Philipp Gahtow (http://pgahtow.de/wiki/index.php?title=DCC#Download)
base on CmdrArduino

===========

* modified by Philipp Gahtow 2015-2021 digitalmoba@arcor.de
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
===========


To install, see the general instructions for Arduino library installation here:
http://arduino.cc/en/Guide/Environment#libraries
