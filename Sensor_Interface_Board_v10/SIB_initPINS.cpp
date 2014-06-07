/*
 * SIB_initPINS.cpp
 *
 *  Created on: May 11, 2014
 *      Author: Tanmay
 */

#include "SIB_initPINS.h"
#include <basictypes.h>
#include <sim.h>
#include <pins.h>


/**********************function to initialize PINS**************************/
void initPINS(){

	uint16_t pwmr=0;
	//Defining the Serial Pins
	J2[3].function(3);//UART0_RX For Debugging
	J2[4].function(3);//UART0_TX

	J2[16].function(2);//UART7_RX For Laser Rangefinder
	J2[20].function(2);//UART7_TX
	J1[7].function(0);
	J1[7]=0;//for continuos data from laser rangefinder
	J2[37].function(0); //Indicator LED
	J2[37] = 0;

	J2[39].function(2);//UART8_RX For Ranging Radio
	J2[42].function(2);//UART8_TX

	J2[41].function(2);//UART9_RX For communication with Nav computer
	J2[44].function(2);//UART9_TX

	J2[23].function(2);//UART5_RX
	J2[24].function(2);//UART5_TX

	//Defining SPI 1 pins
	J2[27].function(1);//SPI_IN
	J2[28].function(1);//SPI_OUT
	J2[30].function(1);//SPI_CS0
	J2[25].function(1);//SPI_SCK

	//Defining IRQ 7
	J2[48].function(1);//IRQ 7 for reading PPM

	//RPGIO pins for Stepper Control
	J2[32].function(0);
	J2[33].function(0);

	//PWM Pins
	J2[40].function(2);//PWM2 B1  3

	//Configuring PWM registers
	//Submodule 1
	sim1.mcpwm.sm[1].cr2=CR2;
	sim1.mcpwm.sm[1].cr1=CR1;
	sim1.mcpwm.sm[1].ocr=OCR;
	sim1.mcpwm.sm[1].dismap=DISMAP;

	//Edge Aligned PWM
	sim1.mcpwm.sm[1].init=P_INIT;
	sim1.mcpwm.sm[1].val[1]=P_MAX;
	sim1.mcpwm.sm[1].val[4]=P_START;
	sim1.mcpwm.sm[1].val[5]=P_END;

	//Enabling PWM outputs
	sim1.mcpwm.outen=OUTEN;
	//starting PWM
	pwmr=sim1.mcpwm.mcr;
	sim1.mcpwm.mcr=LDOK;
	sim1.mcpwm.mcr |=P_RUN;


}
