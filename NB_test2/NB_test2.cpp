/*
 * QuadIMU_NB2_ver2.cpp
 *
 *  Created on: May 16, 2014
 *      Author: Tanmay
 */

//Libraries to be included
#include "predef.h"
#include <stdio.h>
#include <stdlib.h>
#include <basictypes.h> //Defines basic data types
#include <ucos.h> //OS macros
#include <ctype.h>
#include <startnet.h>
#include <autoupdate.h> //Automatic update via ethernet cable
#include <dhcpclient.h>
#include <taskmon.h>
#include <smarttrap.h>
#include <sim.h>
#include <pins.h>
#include <ucosmcfc.h>
#include <pinconstant.h>
#include <HiResTimer.h> //Timer Utility
#include <utils.h>
#include <constants.h>
#include <cfinter.h> //Needed for interrupt
#include <math.h>
#include <serial.h> //Serial macro definitions
#include <dspi.h>	//SPI macro definition
#include <sim5441X.h>  //on-chip register definitions/
#include <ethernet.h>
#include <iosys.h>
#include <bsp.h> //needed for ForceReboot()
#include "QUADIMU_NB2_initPINS.h"

extern "C"{
	void UserMain( void * pd);
	void SetIntc( int intc, long func, int source, int level);
}


void UserMain( void* pd ){
	/////Usual Routine

	InitializeStack();
	OSChangePrio( MAIN_PRIO );
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();

	initPINS();

	SerialClose(7);
	int fdNavcomp=0;
	fdNavcomp=OpenSerial(7,115200,1,8,eParityNone);

	uint8_t commandstatus=0,i=0;
	//uint8_t statestatus=0;
	char Navcomp_in_buff[16]={0};

	while(1){
		i=0;
		ReadWithTimeout(fdNavcomp,&Navcomp_in_buff[i],1,4);
		i++;
		if(Navcomp_in_buff[0]==0x41){
			ReadWithTimeout(fdNavcomp,&Navcomp_in_buff[i],1,4);
			i++;
			if(Navcomp_in_buff[1]==0x7A){
				//iprintf("found header\n");
			for(i=2;i<16;i++){
				ReadWithTimeout(fdNavcomp,&Navcomp_in_buff[i],1,4);
				}//for loop
			}//second if
		}//first if

		commandstatus=(uint8_t)Navcomp_in_buff[7];
		iprintf("%d\n",commandstatus);

	}
}



