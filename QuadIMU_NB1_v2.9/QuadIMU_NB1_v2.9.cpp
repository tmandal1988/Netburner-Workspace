/*
 * QuadIMU_NB1_v2.8.cpp
 *
 *  Created on: Jun 3, 2014
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
#include <pin_irq.h>

extern "C"{
	void UserMain( void * pd);
	void SetIntc( int intc, long func, int source, int level);
}

//IMU Commands:Send values corresponding to the data needed
#define xghigh 0x12
#define xglow  0x10
#define yghigh 0x16
#define yglow  0x14
#define zghigh 0x1A
#define	zglow  0x18
#define	xahigh 0x1E
#define xalow  0x1C
#define	yahigh 0x22
#define	yalow  0x20
#define	zahigh 0x26
#define	zalow  0x24
#define	xm     0x28
#define	ym     0x2A
#define	zm     0x2C
#define	barhigh 0x30
#define	barlow  0x2E

#define INPUT_TASK_STK_SIZE (1024)//autopilot
volatile DWORD INPUTTaskStack[INPUT_TASK_STK_SIZE]__attribute__((aligned( 4 )));//autopilot

BYTE IMU_command[24]={xahigh,0,xalow,0,yahigh,0,yalow,0,zahigh,0,zalow,0,xghigh,0,xglow,0,yghigh,0,yglow,0,zghigh,0,zglow,0};
BYTE IMU1_raw[24]={0};//IMU 3
BYTE IMU2_raw[24]={0};//IMU 4

//Global Variables
/****Millisecond Timer Variables*****/
HiResTimer* timer1;// Netburner ms timer

uint8_t FiftyHzTaskFlag=0;

void initPINS(){

	//GPIO
	J1[6].function(0); //pause on
	J1[7].function(0); //pause off
	J1[13].function(0); //Emergency Light
	J1[10].function(0);
	J2[34].function(0); //NB2 pause interrupt
	J2[48].function(0);//NB1 indicator LED
	J1[6] = 0;J1[7] = 0;J1[10] = 0;J1[13] = 0;J2[34] = 0;J2[48] = 0;//initializing it to low //necessary to activate GPIO out

	J2[19].function(3);//TXD UART2
	J2[31].function(3);//RXD UART2

	//TO PC104
	J2[16].function(2);//uart7 RX
	J2[20].function(2);//uart7 TX

	//Inter NB communication
	J2[39].function(2);//uart8 RX
	J2[42].function(2);//uart8 TX

	//IMU 2
	J2[21].function(1);//SPI3 Input
	J2[22].function(1);//SPI3 Out
	J2[23].function(1);//SPI3 chip select 0
	J2[24].function(1);//SPI3 clock

	//IMU 1
	J2[27].function(1);//SPI1 Input
	J2[28].function(1);//SPI1 Out
	J2[30].function(1);//SPI1 chip select 0
	J2[25].function(1);//SPI1 clock

	//ADC
	J2[6].function(2);
}

void FiftyHzTask(){
	FiftyHzTaskFlag=1;
	J2[48] = J2[48]^1;

}

void initDSPI(){
	DSPIInit(3,2000000,16,0x01,1,1,1,0,0,0);//initializing SPI
	DSPIInit(1,2000000,16,0x01,1,1,1,0,0,0);//initializing SPI

//printf("dspi_stat3=%d,dspi_stat1=%d\n",dspi3_stat,dspi1_stat);
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

//ADC Functions as defined by SimpleAD.h (could not get header file to import properly)
void InitSingleEndAD()
{
  volatile WORD vw;

  //See MCF5441X RM Chapter 29
  sim2.adc.cr1 = 0;
  sim2.adc.cr2 = 0;
  sim2.adc.zccr = 0;
  sim2.adc.lst1 = 0x3210; //Ch 0....
  sim2.adc.lst2 = 0x7654; //ch 7 in result 0..7
  sim2.adc.sdis = 0; //All channels enabled
  sim2.adc.sr = 0xFFFF;
  for (int i = 0; i < 8; i++)
  {
     vw = sim2.adc.rslt[i];
     sim2.adc.ofs[i] = 0;
  }

  sim2.adc.lsr = 0xFFFF;
  sim2.adc.zcsr = 0xFFFF;

  sim2.adc.pwr = 0; //Everything is turned on
  sim2.adc.cal = 0x0000;
  sim2.adc.pwr2 = 0x0005;
  sim2.adc.div = 0x505;
  sim2.adc.asdiv = 0x13;
}

void StartAD()
{
  sim2.adc.sr = 0xffff;
  sim2.adc.cr1 = 0x2000;
}

bool ADDone()
{
  if (sim2.adc.sr & 0x0800)
     return true;
  else
     return false;
}

WORD GetADResult(int ch) //Get the AD Result
{
  return sim2.adc.rslt[ch];
}


void IMU_Acquisition(void *pd){
	uint8_t i = 0;

	while (1){

		if (FiftyHzTaskFlag==1){
			DSPIStart(1,IMU_command,IMU1_raw,24,NULL);//IMU1
			i=0;
			while(!DSPIdone(1) && i<10){i++;/*iprintf("DSPI1done state=%s\n",(DSPIdone(1))?"true":"false");*/};

			DSPIStart(3,IMU_command,IMU2_raw,24,NULL);//IMU1
			i=0;
			while(!DSPIdone(3) && i<10){i++;/*iprintf("DSPI3done state=%s\n",(DSPIdone(3))?"true":"false");*/};
		}
	}
}

void UserMain( void* pd ){
	/////Usual Routine

	InitializeStack();
	OSChangePrio( MAIN_PRIO );
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();

	initPINS();
	initDSPI();

	//Local Variables
	/***********Defining Interrupt Timers*****************/
	HiResTimer* timer2=0;//50 Hz Interrupt Timer
	uint8_t i=0;

	char Navcomp_send_buff[64]={0},time_ms[2]={0};
	uint16_t NB_counter=0,sum=0;
	double TotalTime=0;
	int fdNavcomp=0;
	unsigned short int ADC_channel[8]={0};
	uint8_t pause =0;



	int32_t IMU_data[6]={0};
	uint32_t IMU1_sum=0,IMU2_sum=0,IMU1_sump=0,IMU2_sump=0;

	BYTE IMU_config1[6]={0x80,0x03,0x80,0x61,0x8D,0x00};
	i=0;
	DSPIStart(1,IMU_config1,NULL,6,NULL);//IMU1
	OSTimeDly(1);
	while(!DSPIdone(1) && i<10){i++;};
	DSPIStart(3,IMU_config1,NULL,6,NULL);//IMU2
	OSTimeDly(1);
	i=0;
	while(!DSPIdone(3) && i<10){i++;};

	int j=0;

	Navcomp_send_buff[0] = 0x41;
	Navcomp_send_buff[1] = 0x7A;
	Navcomp_send_buff[2] = 0x03;

	SerialClose(0);
	SerialClose(2);
	SerialClose(7);
	SerialClose(9);

	fdNavcomp=OpenSerial(7,115200,1,8,eParityNone);
	initTIMERS(timer2);
	InitSingleEndAD();

	OSTaskCreate(IMU_Acquisition, NULL,
				(void *) &INPUTTaskStack[INPUT_TASK_STK_SIZE],
				(void *) INPUTTaskStack, MAIN_PRIO+2);

	while(1){
		TotalTime=timer2->readTime();

		if (FiftyHzTaskFlag==1){
			i=0;
			j=0;
			IMU1_sum=0;IMU2_sum=0;
			for(j=0;j<24;j++){
				IMU1_sum+=IMU1_raw[j];
				IMU2_sum+=IMU2_raw[j];
			}

			if(IMU1_sum!=IMU1_sump && IMU2_sum!=IMU2_sump){
				IMU_data[0]=(((int32_t)IMU1_raw[2]<<24|(int32_t)IMU1_raw[3]<<16|(int32_t)IMU1_raw[4]<<8|(int32_t)IMU1_raw[5])-((int32_t)IMU2_raw[2]<<24|(int32_t)IMU2_raw[3]<<16|(int32_t)IMU2_raw[4]<<8|(int32_t)IMU2_raw[5]))/2;//X-Accel
				IMU_data[1]=(((int32_t)IMU1_raw[6]<<24|(int32_t)IMU1_raw[7]<<16|(int32_t)IMU1_raw[8]<<8|(int32_t)IMU1_raw[9])-((int32_t)IMU2_raw[6]<<24|(int32_t)IMU2_raw[7]<<16|(int32_t)IMU2_raw[8]<<8|(int32_t)IMU2_raw[9]))/2;//Y-Accel
				IMU_data[2]=(((int32_t)IMU1_raw[10]<<24|(int32_t)IMU1_raw[11]<<16|(int32_t)IMU1_raw[12]<<8|(int32_t)IMU1_raw[13])+((int32_t)IMU2_raw[10]<<24|(int32_t)IMU2_raw[11]<<16|(int32_t)IMU2_raw[12]<<8|(int32_t)IMU2_raw[13]))/2;//Z-Accel
				IMU_data[3]=(((int32_t)IMU1_raw[14]<<24|(int32_t)IMU1_raw[15]<<16|(int32_t)IMU1_raw[16]<<8|(int32_t)IMU1_raw[17])-((int32_t)IMU2_raw[14]<<24|(int32_t)IMU2_raw[15]<<16|(int32_t)IMU2_raw[16]<<8|(int32_t)IMU2_raw[17]))/2;//X-Gyro
				IMU_data[4]=(((int32_t)IMU1_raw[18]<<24|(int32_t)IMU1_raw[19]<<16|(int32_t)IMU1_raw[20]<<8|(int32_t)IMU1_raw[21])-((int32_t)IMU2_raw[18]<<24|(int32_t)IMU2_raw[19]<<16|(int32_t)IMU2_raw[20]<<8|(int32_t)IMU2_raw[21]))/2;//Y-Gyro
				IMU_data[5]=(((int32_t)IMU1_raw[22]<<24|(int32_t)IMU1_raw[23]<<16|(int32_t)IMU1_raw[0]<<8|(int32_t)IMU1_raw[1])+((int32_t)IMU2_raw[22]<<24|(int32_t)IMU2_raw[23]<<16|(int32_t)IMU2_raw[0]<<8|(int32_t)IMU2_raw[1]))/2;//Z-Gyro
			}

			else{
				if(IMU1_sum!=IMU1_sump && IMU2_sum==IMU2_sump){
					IMU_data[0]=((int32_t)IMU1_raw[2]<<24|(int32_t)IMU1_raw[3]<<16|(int32_t)IMU1_raw[4]<<8|(int32_t)IMU1_raw[5]);//X-Accel
					IMU_data[1]=((int32_t)IMU1_raw[6]<<24|(int32_t)IMU1_raw[7]<<16|(int32_t)IMU1_raw[8]<<8|(int32_t)IMU1_raw[9]);//Y-Accel
					IMU_data[2]=((int32_t)IMU1_raw[10]<<24|(int32_t)IMU1_raw[11]<<16|(int32_t)IMU1_raw[12]<<8|(int32_t)IMU1_raw[13]);//Z-Accel
					IMU_data[3]=((int32_t)IMU1_raw[14]<<24|(int32_t)IMU1_raw[15]<<16|(int32_t)IMU1_raw[16]<<8|(int32_t)IMU1_raw[17]);//X-Gyro
					IMU_data[4]=((int32_t)IMU1_raw[18]<<24|(int32_t)IMU1_raw[19]<<16|(int32_t)IMU1_raw[20]<<8|(int32_t)IMU1_raw[21]);//Y-Gyro
					IMU_data[5]=((int32_t)IMU1_raw[22]<<24|(int32_t)IMU1_raw[23]<<16|(int32_t)IMU1_raw[0]<<8|(int32_t)IMU1_raw[1]);//Z-Gyro

					//printf("%g,%d,%g\n",0.00025*((int32_t)IMU1_raw[10]<<24|(int32_t)IMU1_raw[11]<<16|(int32_t)IMU1_raw[12]<<8|(int32_t)IMU1_raw[13])/65536,k,0.00025*IMU_data_sum[2]/65536);
				}

				if(IMU2_sum!=IMU2_sump && IMU1_sum==IMU1_sump){
					IMU_data[0]=-((int32_t)IMU2_raw[2]<<24|(int32_t)IMU2_raw[3]<<16|(int32_t)IMU2_raw[4]<<8|(int32_t)IMU2_raw[5]);//X-Accel
					IMU_data[1]=-((int32_t)IMU2_raw[6]<<24|(int32_t)IMU2_raw[7]<<16|(int32_t)IMU2_raw[8]<<8|(int32_t)IMU2_raw[9]);//Y-Accel
					IMU_data[2]=((int32_t)IMU2_raw[10]<<24|(int32_t)IMU2_raw[11]<<16|(int32_t)IMU2_raw[12]<<8|(int32_t)IMU2_raw[13]);//Z-Accel
					IMU_data[3]=-((int32_t)IMU2_raw[14]<<24|(int32_t)IMU2_raw[15]<<16|(int32_t)IMU2_raw[16]<<8|(int32_t)IMU2_raw[17]);//X-Gyro
					IMU_data[4]=-((int32_t)IMU2_raw[18]<<24|(int32_t)IMU2_raw[19]<<16|(int32_t)IMU2_raw[20]<<8|(int32_t)IMU2_raw[21]);//Y-Gyro
					IMU_data[5]=((int32_t)IMU2_raw[22]<<24|(int32_t)IMU2_raw[23]<<16|(int32_t)IMU2_raw[0]<<8|(int32_t)IMU2_raw[1]);//Z-Gyro
				}

				if(IMU2_sum==IMU2_sump && IMU1_sum==IMU1_sump){
					IMU_data[0]=IMU_data[0];
					IMU_data[1]=IMU_data[1];
					IMU_data[2]=IMU_data[2];
					IMU_data[3]=IMU_data[3];
					IMU_data[4]=IMU_data[4];
					IMU_data[5]=IMU_data[5];
				}
			}

				IMU1_sump=IMU1_sum;
				IMU2_sump=IMU2_sum;

			//printf("Hi\n");
			//packaging NB counter
			Navcomp_send_buff[4]=(uint8_t)((NB_counter & 0xFF00)>>8);
			Navcomp_send_buff[3]=(uint8_t)(NB_counter & 0x00FF);
			NB_counter++;

			//getting time in ms
			printf("Z-Accel=%g\n",0.00025*IMU_data[2]/65536);

			//printf("%g\n",0.00025*IMU_data[2]/65536);
			sprintf(time_ms,"%lf",TotalTime);
			Navcomp_send_buff[6]=time_ms[0];
			Navcomp_send_buff[5]=time_ms[1];


			i=0;
			for(i=0;i<6;i++){
				Navcomp_send_buff[4*i+10]=(BYTE)((uint32_t)(IMU_data[i] & 0xFF000000)>>24);
				Navcomp_send_buff[4*i+9]=(BYTE)((uint32_t)(IMU_data[i] & 0x00FF0000)>>16);
				Navcomp_send_buff[4*i+8]=(BYTE)((uint32_t)(IMU_data[i] & 0x0000FF00)>>8);
				Navcomp_send_buff[4*i+7]=(BYTE)((uint32_t)(IMU_data[i] & 0x000000FF));

			}

			StartAD();
			while (!ADDone()){}
			asm("nop");
			i=0;
			for (i = 0; i < 8; i++)
				ADC_channel[i] = (unsigned short int)(1000 * (((double)GetADResult(i)) * 3.3 / (32768.0)));

			i=0;
			for(i=0;i<8;i++){
				Navcomp_send_buff[32+2*i]=(uint8_t)((ADC_channel[i] & 0xFF00)>>8);
				Navcomp_send_buff[31+2*i]=(uint8_t)(ADC_channel[i] & 0x00FF);
			}


			//Remote Pause
			if(J1[6].read())
				pause = 0xFF;
			if(J1[7].read())
				pause = 0;

			//Emergency Light & NB2 interrupt
			if (NB_counter%50 == 0 && pause == 0)
				J1[13] = J1[13]^1;
			if (pause == 0);
				J2[34] = 0;
			if (pause == 0xFF){
				J1[13] = 1;
				J2[34] = 1;
			}

			//printf("%d %d %d\n", (int)J1[10].read(), pause, NB_counter);

			Navcomp_send_buff[47] = pause;

			//Calculating checksum
			sum=0;
			i=0;
			for(i=3;i<63;i++){
				sum +=Navcomp_send_buff[i];
			}

			Navcomp_send_buff[63]=(uint8_t)(sum % 256);

			i=0;
			for(i=0;i<sizeof(Navcomp_send_buff);i++){
				write(fdNavcomp,&Navcomp_send_buff[i],1);
			}

			FiftyHzTaskFlag=0;

		}//Fifty Hz if
	}//Main While Loop
}


