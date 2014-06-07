/*
 * QUADIMU_NB2_initPINS.cpp
 *
 *  Created on: May 11, 2014
 *      Author: Tanmay
 */

#include "QUADIMU_NB2_initPINS.h"
#include <basictypes.h>
#include <sim.h>
#include <pins.h>


/**********************function to initialize PINS**************************/
void initPINS(){

	//Defining the Serial Pins
	J2[3].function(3);//UART0_RX For Debugging
	J2[4].function(3);//UART0_TX

	J2[19].function(3);//UART 2 for grabber
	J2[31].function(3);

	J2[41].function(2);//UART9_RX for Crank
	J2[44].function(2);//UART9_TX

	J2[16].function(2);//UART7 for communicating with Navcomp
	J2[20].function(2);

	//IMU 4
	J2[21].function(1);//SPI3 Input
	J2[22].function(1);//SPI3 Out
	J2[23].function(1);//SPI3 chip select 0
	J2[24].function(1);//SPI3 clock

	//IMU 3
	J2[27].function(1);//SPI1 Input
	J2[28].function(1);//SPI1 Out
	J2[30].function(1);//SPI1 chip select 0
	J2[25].function(1);//SPI1 clock

	J2[45].function(0);//Pause Pin
}


