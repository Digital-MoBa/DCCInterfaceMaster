/*
 * Z21type.h
 * Created on: 16.04.2015
 *
 * Copyright (c) by Philipp Gahtow, year 2015
*/
//**************************************************************
//Client Configuration:

//Speichergröße:
#if defined(__AVR_ATmega1284P__)
#define LANmaxIP 15     //max IP-Adressen (max Clients)
#define WLANmaxIP 15    //Anzahl Clients über ESP
#elif defined(MEGA_MCU)  //Arduino MEGA
#define LANmaxIP 15
#define WLANmaxIP 15    //Anzahl Clients über ESP
#elif defined (DUE_MCU)
#define LANmaxIP 20
#define WLANmaxIP 20    //Anzahl Clients über ESP
#elif defined (__AVR_ATmega644P__)
#define LANmaxIP 15
#define WLANmaxIP 15    //Anzahl Clients über ESP
#else   //Arduino UNO
#define LANmaxIP 5
#define WLANmaxIP 10    //Anzahl Clients über ESP
#endif

#define ActTimeIP 10    //Aktivhaltung einer IP für (sec./2)
#define IPinterval 4000   //interval in milliseconds for checking IP aktiv state
