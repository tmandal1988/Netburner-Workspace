/*
 * Sensor_Interface_Board_v9.cpp
 *
 *  Created on: June 6, 2014
 *      Author: Tanmay Kumar Mandal, Email: tmandal1988@gmail.com
 *      This Code Gets
 *      1. Range Data from Ranging Radio through TTL UART8
 *      2. Analog data through ADC-Scott
 *      3. Communicate with mission and NAV computer via RS232 UART9-finished
 *      5. Control PAN head using B1 PWM module-Scott
 *   	6. Read Laser Rangefinder through TTL UART7-finished not used
 *
 *      Number of Interrupts 1
 *      1. 50 Hz Interrupt to read Laser rangefinder and send data packet to the computer-finished
 *      	*It also has a counter which counts till 10 to simulate a 5 Hz Interrupt.-finished
 *
 *      Number of Tasks  2
 *      1. Main Task-finished
 *      2. Receive serial buffer from the Nav/Mission Computer-finished
 *
 *      Modified By: Scott Added ADC
 *
 *      This code sends data irrespective of whether it is getting data from the computer or not.
 *      Ranging Radio Comm-Debugged
 *
 *      Warning!!!!!: UART4 and UART0 won't work together when openserial macro is used
 *
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
#include "SimpleAD.h"///including header file for ADC peripheral
#include "Ranging_Radio.h"
#include "Laser_rangefinder.h"
#include "SIB_SendtoNAVCOMP.h"
#include "SIB_initPINS.h"

//Tells the compiler its a C code wrapped in C++
extern "C"{
 void UserMain( void * pd);
 void SetIntc( int intc, long func, int source, int level);
}

///Gimbal PAN angle
static int16_t Start_PAN=0;
static uint8_t Scan_Status=0,Scan_Complete=0;
/**************PAN variable**************************/
static uint16_t Pulse = 12287;
uint16_t pwmr_comp=0;
double dYaw=0;
static int fdRadio=0;
static unsigned char F_range_buff[7]={0};//Radio filtered range
static uint8_t Ant_config=0;
int16_t PanAngle=0;

//Global Variables
/****Millisecond Timer Variables*****/
HiResTimer* timer1;// Netburner ms timer

/***Flag Variables****/
uint8_t FiftyHzflag=0,FiveHzflag=0;//Variable flag in the FiftyHzInterrupt and FiveHzInterrupt
uint8_t FiveHzcount=0;


/*****File Descriptor for NAVcomp communication and receive buffer**************/
static int fdNAVcomp=0;static char Navcomp_in_buff[32]={0};

/**************MIS comp communication*******************************************/
static char Navcomp_in_buffMIS[32]={23};

/*********File Descriptor for MIS communication and receive buffer**************/
static int fdDebug=0;

/*******laser Range file descriptor*********************/
 static int fdLaser=0;




/*********Service Watchdog function**************/
void serviceWatchDog(){
	sim2.scm.cwsr = 0x55;
	sim2.scm.cwsr = 0xAA;
}

/************************Task to receive data from Mission Comp***************/
void MIScompData(void *){
	uint8_t i=0;
	uint16_t k=0;
	char m=0;
	int com_timeout=0;
	//uint16_t sum=0;
	//uint8_t checksum=0;
	/**************PAN variable**************************/
	while(1){

		i=0;

		ReadWithTimeout(fdDebug,&Navcomp_in_buffMIS[i],1,1);
		//printf("%d\n",stat);
		i++;
		if(Navcomp_in_buffMIS[0]==0x41){
			//("%g\n",dYaw);
			ReadWithTimeout(fdDebug,&Navcomp_in_buffMIS[i],1,1);

			i++;
			ReadWithTimeout(fdDebug,&Navcomp_in_buffMIS[i],1,1);
			i++;
			if(Navcomp_in_buffMIS[1]==0x7A && Navcomp_in_buffMIS[2]==0x07)
				{	//ReadWithTimeout(fdNAVcomp,&Navcomp_in_buff[i],1,1);
				//printf("here\n");
					i=3;
					while(i<32){
						ReadWithTimeout(fdDebug,&Navcomp_in_buffMIS[i],1,4);
						i++;
					}//for loop
					/*sum=0;
					checksum=0;

					for(i=3;i<(sizeof(Navcomp_in_buff)-1);i++)
						sum +=Navcomp_in_buff[i];

						checksum=(uint8_t)sum%255;*/

					//printf("Hi and mode=%d\n",(unsigned char)Navcomp_in_buffMIS[19]);

					if((unsigned char)Navcomp_in_buffMIS[19]==1){
						Scan_Complete=0;
						Start_PAN=StartUpLaserScan(fdLaser);
						//ujgts6printf("Hi and mode=%d\n",(unsigned char)Navcomp_in_buffMIS[19]);
						if(Start_PAN!=11110)
							Scan_Status=1;
						else{
							Scan_Status=0;
							Start_PAN=11110;
						}
						Scan_Complete=1;

						//printf("Hi2 and mode=%d\n",(unsigned char)Navcomp_in_buffMIS[19]);

					}
					//printf("Hi3 and mode=%d\n",(unsigned char)Navcomp_in_buffMIS[19]);

					//serviceWatchDog();

				}//second if

		}//first if*///Task main while loop

		k=0;
		for(k=0;k<256;k++){
			com_timeout=ReadWithTimeout(fdDebug,&m,1,1);
			if(com_timeout==0 || com_timeout==-1){
				break;
			}
		}
	}//While loop

}//MIScompData task bracket
/************************Task to receive data from Nav Comp******************/
void NAVcompData(void *){
	uint8_t i=0;
	uint16_t sum=0;
	uint8_t checksum=0;



	while(1){

		i=0;

		ReadWithTimeout(fdNAVcomp,&Navcomp_in_buff[i],1,1);
		//printf("%d\n",stat);
		i++;
		if(Navcomp_in_buff[0]==0x41){
			//("%g\n",dYaw);
			ReadWithTimeout(fdNAVcomp,&Navcomp_in_buff[i],1,1);

			i++;
			ReadWithTimeout(fdNAVcomp,&Navcomp_in_buff[i],1,1);
			i++;
			if(Navcomp_in_buff[1]==0x7A && Navcomp_in_buff[2]==0x07)
				{	//ReadWithTimeout(fdNAVcomp,&Navcomp_in_buff[i],1,1);
				//printf("here\n");
					i=3;
					while(i<32){
						ReadWithTimeout(fdNAVcomp,&Navcomp_in_buff[i],1,4);
						i++;
					}//for loop
					sum=0;
					checksum=0;

					for(i=3;i<(sizeof(Navcomp_in_buff)-1);i++)
						sum +=Navcomp_in_buff[i];

						checksum=(uint8_t)sum%255;

					if((uint8_t)checksum==(uint8_t)Navcomp_in_buff[31]){

					//if(Scan_Complete==1){
						//Pulse =12287-20.51*double((int16_t)((uint8_t)Navcomp_in_buff[18] * 256 + (uint8_t)Navcomp_in_buff[17]))/10;
						dYaw  = double((int16_t)((uint8_t)Navcomp_in_buff[18] * 256 + (uint8_t)Navcomp_in_buff[17]))/10;
					}

					//serviceWatchDog();

				}//second if

		}//first if*/
	}//while

}//process NAVcompData

/////Function Definitions


/**********************50 Hz Task Function******************************/
void FiftyHzTask(){


	FiftyHzflag=1;

	/***********Five Hz Routine***************/
	FiveHzcount +=1;
	if(FiveHzcount==9){
		FiveHzflag =1;
		FiveHzcount=0;
	}

}


/*******************function to initialize Timers*********************/
void initTIMERS(HiResTimer* timer2){
	timer1=HiResTimer::getHiResTimer(1);
	timer1->init();
	timer1->start();

	timer2=HiResTimer::getHiResTimer(0);
	timer2->setInterruptFunction(FiftyHzTask);
	timer2->init(0.02);
	timer2->start();
}

/******Enable Watchdog function***********************************/
int enableWatchDog( bool readOnly, int timeoutCycles ){

	unsigned short mask = 0;

	if ( readOnly )
		mask |= 0x8000; // set RO bit 15

	if ( (timeoutCycles < 8) || (timeoutCycles > 0x001F) )
		return -1;
	else
		mask |= timeoutCycles; // cycles are bits 4-0

	mask |= 0x00C0; // bit 7 = enable, bits 6-5 = generate system reset on timeout

	//iprintf("Writing CWCR: 0x%04X\r\n", mask );
	sim2.scm.cwcr = mask;

	return 0;
}

void RadioData(void *){

	uint16_t CRME=0;

	unsigned char* radio_in_buff=0;
	/************Ranging Radio Commands buffer************/
	char RCM_SEND_RANGE_REQUESTA1[]={0xA5,0xA5,0x00,0x0D,0x00,0x03,0x00,0x01,0x00,0x00,0x00,0x65,0x00,0x00,0x00,0x01,0x00,0x45,0xF2};//Host Antenna A to 101
	//char RCM_SEND_RANGE_REQUESTA2[]={0xA5,0xA5,0x00,0x0D,0x00,0x03,0x00,0x02,0x00,0x00,0x00,0x66,0x00,0x00,0x00,0x01,0x00,0x3A,0xDD};//Host Antenna A to 102
	while(1){

		radio_in_buff=ReadRadio(RCM_SEND_RANGE_REQUESTA1,fdRadio,sizeof(RCM_SEND_RANGE_REQUESTA1));
		//radio_in_buff2=ReadRadio(RCM_SEND_RANGE_REQUESTA2,fdRadio,sizeof(RCM_SEND_RANGE_REQUESTA2));
		//printf("%d\n",(uint8_t)radio_in_buff11[11]);

		/*if(Radio_mail_rec==NULL){
		//printf("%d\n",(uint8_t)radio_in_buff11[11]);*/
			CRME=(uint16_t)radio_in_buff[28]*256+(uint16_t)radio_in_buff[29];
			if((unsigned char)radio_in_buff[12]==0 && CRME<60){
				F_range_buff[0]=radio_in_buff[24];
				F_range_buff[1]=radio_in_buff[25];
				F_range_buff[2]=radio_in_buff[26];
				F_range_buff[3]=radio_in_buff[27];
				F_range_buff[4]=radio_in_buff[32];
				F_range_buff[5]=radio_in_buff[33];
				F_range_buff[6]=radio_in_buff[12];
				Ant_config=radio_in_buff[11];
				//printf("%d\n",Ant_config);
			}
		}



}//Radio Data Task

///////////////////////////////////MAIN FUNCTION//////////////////////////////////////////////
void UserMain( void* pd ){

	/////Usual Routine
	InitializeStack();
	OSChangePrio( MAIN_PRIO );//Change Main Task number to MAIN_PRIO
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();

	//serviceWatchDog();

	//Starting the Code
	iprintf("\n\n\n..................Starting Sensor Interface Board.....................\n\n\n");



	//Local Variables
	//////Scan Flag

	/***********Defining Interrupt Timers*****************/
	HiResTimer* timer2=0;//50 Hz Interrupt Timer

	/***********File Descriptor Variables****************/
	int fduart0;



	/**********Radio Debug Variable*******************/
	double TotalTime=0;	char time_ms[2]={0};
	/***********Radio control Radiocount and loop counter i********************/
	uint8_t Radiocount3=0;



	/*********Laser Rangefinder Variables***************/
	float laser_range=0;


	/**********ADC channel Array***************************/
	uint16_t ADC_channel[8] = {0},i=0;

	/**********Navcomp send buffer and other vriables**********************/
	char Navcomp_send_buff[48]={0};
	Navcomp_send_buff[0]=0x41;
	Navcomp_send_buff[1]=0x7A;
	Navcomp_send_buff[2]=0x05;
	uint16_t netburner_counter=0;//netburner 16 bit counter

	//Initialize pins
	initPINS();

	//Initialize Analog to Digital
	InitSingleEndAD();

	//Initializing Serial Ports
	SerialClose(5);
	SerialClose(7);
	SerialClose(9);
	SerialClose(8);
	SerialClose(0);


	fdRadio= OpenSerial( 8, 115200, 1, 8, eParityNone );
	fdDebug = OpenSerial( 5, 115200, 1, 8, eParityNone );
	fdLaser = OpenSerial( 7, 115200, 1, 8, eParityNone );
	fdNAVcomp = OpenSerial( 9, 115200, 1, 8, eParityNone );
	fduart0 = OpenSerial( 0, 115200, 1, 8, eParityNone );

	/************Ranging Radio Commands buffer************/
	char RCM_SEND_RANGE_REQUESTA1[]={0xA5,0xA5,0x00,0x0D,0x00,0x03,0x00,0x01,0x00,0x00,0x00,0x65,0x00,0x00,0x00,0x01,0x00,0x45,0xF2};//Host Antenna A to 101
	char RCM_SEND_RANGE_REQUESTA2[]={0xA5,0xA5,0x00,0x0D,0x00,0x03,0x00,0x02,0x00,0x00,0x00,0x66,0x00,0x00,0x00,0x01,0x00,0x3A,0xDD};//Host Antenna A to 102

	/*
	ReplaceStdio(0,fdDebug);
	ReplaceStdio(1,fdDebug);
	ReplaceStdio(2,fdDebug);
	*/

	//Start the Timers and init the DSPI
	//DSPIInit(1,2000000,16,0x01,0x01,1,1,0,0,0);//initializing SPI

	//printf("Going to wait 3 sec\n");
	//OSTimeDly(3*TICKS_PER_SECOND);
	initTIMERS(timer2);
	J1[7]=0;
	/*startup_timeout=ReadWithTimeout(fdDebug,&m,1,2);
	if(startup_timeout==-1 || startup_timeout==0){
		Start_PAN=StartUpLaserScan(fdLaser);
		if(Start_PAN!=11110)
			Scan_Status=1;
		else
			Scan_Status=0;
	}

	else{
		Scan_Status=0;
		Start_PAN=11110;
	}*/

	//printf("Hi\n");
	Scan_Complete=1;
	/***********packing startup PAN angle*************************/
	Navcomp_send_buff[38] = (uint8_t)((Start_PAN & 0xFF00)>>8);
	Navcomp_send_buff[37] = (uint8_t)(Start_PAN & 0x00FF);
	Navcomp_send_buff[36] = Scan_Status;
	OSSimpleTaskCreate(NAVcompData,MAIN_PRIO-1);
	OSSimpleTaskCreate(RadioData,MAIN_PRIO-2);
	//OSSimpleTaskCreate(RadioData2,MAIN_PRIO-3);
	//OSSimpleTaskCreate(MIScompData,MAIN_PRIO+2);
	//enableWatchDog( 1, 0x001F );//0x001C
	//Creating Data Receiving task from the computer

	while(1){

		//printf("Hi\n");
		TotalTime=timer1->readTime();

		//First if statement to command host radio to get ranging data from 101 guest with antenna A
		if(FiveHzflag==1 && Radiocount3==0){
			i=0;
			for (i=0;i<sizeof(RCM_SEND_RANGE_REQUESTA1);i++){
				write(fdRadio,&RCM_SEND_RANGE_REQUESTA1[i],1);//Sending ranging command to the host radio
			}
			//printf("In WHile Loop\n");

			Radiocount3=1;
			FiveHzflag=0;
		}//first if bracket

		//second if statement to command host radio to get ranging data from 102 guest with antenna A
		if(FiveHzflag==1 && Radiocount3==1){

			i=0;
			for (i=0;i<sizeof(RCM_SEND_RANGE_REQUESTA2);i++){
				write(fdRadio,&RCM_SEND_RANGE_REQUESTA2[i],1);//Sending ranging command to the host radio
			}

			Radiocount3=0;
			FiveHzflag=0;
		}//second if bracket

		if(FiftyHzflag==1){

			//printf("%d\n",(uint8_t)radio_in_buff11[11]);
			J2[37] = J2[37]^1;

			if(Scan_Complete==1){
			//printf("Start Pan=%d,Start Pan=%d\n",Start_PAN,(int16_t)Navcomp_send_buff[38]*256+(int16_t)Navcomp_send_buff[37]);
				//laser_range=ReadLaser(fdLaser);
				Navcomp_send_buff[36] = Scan_Status;
				Navcomp_send_buff[38] = (uint8_t)((Start_PAN & 0xFF00)>>8);
				Navcomp_send_buff[37] = (uint8_t)(Start_PAN & 0x00FF);
				//printf("laser range=%g\n",laser_range);
				//printf("laser range=%g\n",laser_range);
				//uint32_t Range=(uint32_t)F_range_buff[0]*16777216+(uint32_t)F_range_buff[1]*65536+(uint32_t)F_range_buff[2]*256+(uint32_t)F_range_buff[3];

				//printf("%zu,%d\n",Range,Ant_config);


				StartAD();
				while (!ADDone()){}
				asm("nop");
				for (int i = 0; i < 8; i++)
					ADC_channel[i] = (unsigned short int)(1000 * (((double)GetADResult(i)) * 3.3 / (32768.0)));

				//printf("%d \n", ADC_channel[1]);

				sprintf(time_ms,"%lf",TotalTime);

				//send data to the computer
				SendtoNAVCOMP(Navcomp_send_buff,ADC_channel,time_ms,netburner_counter,laser_range,F_range_buff,PanAngle,fdNAVcomp,fdDebug,sizeof(Navcomp_send_buff),Ant_config);
				netburner_counter ++;


				//printf("%g\n",dYaw);
				//dYaw=0;

				StartAD();
				 while (!ADDone()){}
				asm("nop");

				//dYaw=93;

				uint16_t ServoPot = GetADResult(0);
				////Servo PAN 1 numbers
				Pulse=12287-dYaw*20.51;
				//printf("%d\n",ServoPot);

				if(Pulse<8594 || Pulse==8594)
					sim1.mcpwm.sm[1].val[5]=8594;
				if(Pulse>15980 || Pulse==15980)
					sim1.mcpwm.sm[1].val[5]=15980;
				else
					sim1.mcpwm.sm[1].val[5]=Pulse;//PAN control

				////Servo PAN 2 numbers
				/*Pulse=12287-dYaw*20.14;

				if(Pulse<8594 || Pulse==8310)
					sim1.mcpwm.sm[1].val[5]=8310;
				if(Pulse>15980 || Pulse==15560)
					sim1.mcpwm.sm[1].val[5]=15560;
				else
					sim1.mcpwm.sm[1].val[5]=Pulse;//PAN control*/

				double cYaw=(ServoPot-12885)/63;//PAN 1
				//double cYaw=(ServoPot-14667)/63.05;//PAN 2

				//Calibration PAN servo 1
				//0-8594
				//90-10440
				//180-12287 Position in which PAN faces front
				//270-14844
				//360-15980

				//Calibration pot PAN servo 1
				//360=1564
				//180=12885 //position in which PAN faces front
				//270=7270
				//90=18560
				//0=24290

				//Calibration PAN servo 2
				//0-8310
				//90-10440
				//180-12287 Position in which PAN faces front
				//270-13900
				//360-15560

				//Calibration pot PAN servo 2
				//360=3264
				//180=14667 //position in which PAN faces front
				//270=8999
				//90=20085
				//0=25971

				pwmr_comp=sim1.mcpwm.mcr;
				sim1.mcpwm.mcr |=LDOK;

				PanAngle = cYaw * 10;
				//printf("%g\n",cYaw);


			}
				FiftyHzflag=0;

			//serviceWatchDog();//
		}//FiftyHzflag bracket

	}//Main While loop Bracket

}//Main Loop Bracket





