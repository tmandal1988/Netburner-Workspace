/*
 * QuadIMU_NB2_ver2.cpp
 *
 *  Created on: May 16, 2014
 *      Author: Tanmay Kumar Mandal, Email: tmandal1988@gmail.com
 *      Co-Author: Scott Harper
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



extern "C"{
	void UserMain( void * pd);
	void SetIntc( int intc, long func, int source, int level);
}


int fdNavcomp=0;
char Nav_in_buff[16]={0};
int fdcrank=0,fdgrabber=0;
static uint8_t statestatus=0;
char Navcomp_send_buff[64]={0};
uint8_t FiftyHzTaskFlag=0;

static uint16_t NB_counter=0;

void NAVcompData(void *){
	char m=0;
	uint16_t i=0;
	char grab_interrupt = 0;
	HiResTimer* timer;
	timer = HiResTimer::getHiResTimer();
	char G[20]={0};
	char G1[20]={0};
	char G2[20]={0};
	sprintf(G1,"!EX\r");
	sprintf(G2,"!MG\r");

	while(1){
		read(fdNavcomp,&m,1);
		//printf("%d\n",(uint8_t)m);
		if(m==0x41){
			read(fdNavcomp,&m,1);
			Nav_in_buff[1]=m;
			read(fdNavcomp,&m,1);
			Nav_in_buff[2]=m;
			if(Nav_in_buff[1]==0x7A && Nav_in_buff[2]==0x06){
				i=3;
				for(i=3;i<16;i++)
					read(fdNavcomp,&Nav_in_buff[i],1);
			}

		}

		sprintf(G,"!VAR 1 %d\r",(uint8_t)Nav_in_buff[7]);
		i=0;
		for (i=0;i<sizeof(G);i++){
				write(fdcrank,&G[i],1);
				write(fdgrabber,&G[i],1);
		}

		sprintf(G,"!VAR 3 %d\r",(uint8_t)Nav_in_buff[8]);
		i=0;
		for (i=0;i<sizeof(G);i++){
				write(fdcrank,&G[i],1);
				write(fdgrabber,&G[i],1);
		}

		//Remote Pause
		if (J2[45].read() == 1 && grab_interrupt == 0){

			i=0;
			for (i=0;i<sizeof(G1);i++){
					write(fdcrank,&G1[i],1);
					write(fdgrabber,&G1[i],1);
			}

			i=0;
			for (i=0;i<sizeof(G1);i++){
					write(fdcrank,&G1[i],1);
					write(fdgrabber,&G1[i],1);
			}

			i=0;
			for (i=0;i<sizeof(G1);i++){
					write(fdcrank,&G1[i],1);
					write(fdgrabber,&G1[i],1);
			}

			i=0;
			for (i=0;i<sizeof(G1);i++){
					write(fdcrank,&G1[i],1);
					write(fdgrabber,&G1[i],1);
			}
			//printf("pause on\n");
			grab_interrupt = 1;
		}
		if (J2[45].read() == 0 && grab_interrupt == 1){

			i=0;
			for (i=0;i<sizeof(G2);i++){
					write(fdcrank,&G2[i],1);
					write(fdgrabber,&G2[i],1);
			}

			i=0;
			for (i=0;i<sizeof(G2);i++){
					write(fdcrank,&G2[i],1);
					write(fdgrabber,&G2[i],1);
			}

			i=0;
			for (i=0;i<sizeof(G2);i++){
					write(fdcrank,&G2[i],1);
					write(fdgrabber,&G2[i],1);
			}

			i=0;
			for (i=0;i<sizeof(G2);i++){
					write(fdcrank,&G2[i],1);
					write(fdgrabber,&G2[i],1);
			}
			//printf("pause off\n");
			grab_interrupt = 0;
		}

		OSTimeDly(1);
		//printf("%d\n",(uint8_t)Nav_in_buff[7]);
	}
}

void FiftyHzTask(){

		FiftyHzTaskFlag=1;
		//J2[48] = J2[48]^1;
		if (NB_counter%50 == 0)
		{
			J2[48] = J2[48] ^ 1;
			//J1[13] = 1;
			//toggle = 1;
			//printf("on \n");
		}

		NB_counter++;
}


void ESCstatus(void *){
	char G3[20]={0};
	uint8_t i=0;
	uint8_t stat=0;
	sprintf(G3,"?VAR 4\r");
	char m=0;
	char in_buff[20]={0};

	while(1){

		i=0;
		for (i=0;i<sizeof(G3);i++){
			write(fdcrank,&G3[i],1);

		}

		m=0;
		i=0;
		while(m!=0x3D && i<30){
			//printf("statestatus=%d\n",(unsigned char)m1);
			ReadWithTimeout(fdcrank,&m,1,4);
			i++;
			//if(timeout==-1 || timeout==0)
				//break;
		}

		m=0;
		i=0;
		while(m!=0x0D && i<30){
			//printf("statestatus=%d\n",(unsigned char)m1);
			ReadWithTimeout(fdcrank,&m,1,4);
			in_buff[i]=m;
			i++;
			//if(timeout==-1 || timeout==0)
				//break;
		}


		stat=(uint8_t)atoi(in_buff);
		if(stat==97 || stat==91 || stat==101)
			statestatus=stat;

		OSTimeDly(1);

	}


}


void initDSPI(){
	DSPIInit(3,2000000,16,0,0,1,1,0,0,0);//initializing SPI IMU4
	DSPIInit(1,2000000,16,0,0,1,1,0,0,0);//initializing SPI IMU3
}

void Send_Data(void* pd){
	uint8_t i=0;
	while(1){
		i=0;
		for(i=0;i<sizeof(Navcomp_send_buff);i++){
			write(fdNavcomp,&Navcomp_send_buff[i],1);
			//printf("%d\n",i);
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

	char G[20]={0};
	int32_t IMU_data[6]={0};
	uint32_t IMU3_sum=0,IMU4_sum=0,IMU3_sump=0,IMU4_sump=0;
	uint16_t NB_counter=0,sum=0;

	BYTE IMU_command[24]={xahigh,0,xalow,0,yahigh,0,yalow,0,zahigh,0,zalow,0,xghigh,0,xglow,0,yghigh,0,yglow,0,zghigh,0,zglow,0};
	BYTE IMU3_raw[24]={0};//IMU 3
	BYTE IMU4_raw[24]={0};//IMU 4

	uint8_t i=0,j=0;


	Navcomp_send_buff[0] = 0x41;
	Navcomp_send_buff[1] = 0x7A;
	Navcomp_send_buff[2] = 0x04;

	SerialClose(7);

	fdNavcomp=OpenSerial(7,115200,1,8,eParityNone);
	fdcrank=OpenSerial(9,115200,1,8,eParityNone);
	fdgrabber=OpenSerial(2,115200,1,8,eParityNone);


	OSSimpleTaskCreate(NAVcompData,MAIN_PRIO-2)
	OSSimpleTaskCreate(ESCstatus,MAIN_PRIO-3);
	//OSSimpleTaskCreate(Send_Data,MAIN_PRIO-1);

	BYTE IMU_config1[6]={0x80,0x03,0x80,0x61,0x8D,0x00};//SET IMU DEC value to 98
	i=0;
	DSPIStart(1,IMU_config1,NULL,6,NULL);//IMU3
	OSTimeDly(1);
	while(!DSPIdone(1) && i<10){i++;};
	DSPIStart(3,IMU_config1,NULL,6,NULL);//IMU4
	OSTimeDly(1);
	i=0;
	while(!DSPIdone(3) && i<10){i++;};




	sprintf(G,"~ECHOF 1\r");//turning of the ECHO from ESC
	i=0;
	for (i=0;i<sizeof(G);i++){
			write(fdcrank,&G[i],1);
			write(fdgrabber,&G[i],1);
	}


	//Local Variables
	/***********Defining Interrupt Timers*****************/
	HiResTimer* timer2=0;//50 Hz Interrupt Timer
	timer2=HiResTimer::getHiResTimer(0);
	timer2->setInterruptFunction(FiftyHzTask);
	timer2->init(0.02);
	timer2->start();

	HiResTimer* timer3=0;
	timer3=HiResTimer::getHiResTimer();

	while(1){




		DSPIStart(1,IMU_command,IMU3_raw,24,NULL);//IMU1
		DSPIStart(3,IMU_command,IMU4_raw,24,NULL);//IMU3
		timer3->delay(0.019);
		//OSTimeDly(1);

		i=0;
		while(!DSPIdone(1) && i<10){i++;/*iprintf("DSPI1done state=%s\n",(DSPIdone(1))?"true":"false");*/};

		i=0;
		while(!DSPIdone(3) && i<10){i++;/*iprintf("DSPI3done state=%s\n",(DSPIdone(3))?"true":"false");*/};

		Navcomp_send_buff[47]=statestatus;



		j=0;
		IMU3_sum=0;IMU4_sum=0;
		for(j=0;j<24;j++){
			IMU3_sum+=IMU3_raw[j];
			IMU4_sum+=IMU4_raw[j];
		}

		if(IMU3_sum!=IMU3_sump && IMU4_sum!=IMU4_sump){
			IMU_data[0]=(((int32_t)IMU3_raw[2]<<24|(int32_t)IMU3_raw[3]<<16|(int32_t)IMU3_raw[4]<<8|(int32_t)IMU3_raw[5])-((int32_t)IMU4_raw[2]<<24|(int32_t)IMU4_raw[3]<<16|(int32_t)IMU4_raw[4]<<8|(int32_t)IMU4_raw[5]))/2;//X-Accel
			IMU_data[1]=(((int32_t)IMU3_raw[6]<<24|(int32_t)IMU3_raw[7]<<16|(int32_t)IMU3_raw[8]<<8|(int32_t)IMU3_raw[9])-((int32_t)IMU4_raw[6]<<24|(int32_t)IMU4_raw[7]<<16|(int32_t)IMU4_raw[8]<<8|(int32_t)IMU4_raw[9]))/2;//Y-Accel
			IMU_data[2]=(((int32_t)IMU3_raw[10]<<24|(int32_t)IMU3_raw[11]<<16|(int32_t)IMU3_raw[12]<<8|(int32_t)IMU3_raw[13])+((int32_t)IMU4_raw[10]<<24|(int32_t)IMU4_raw[11]<<16|(int32_t)IMU4_raw[12]<<8|(int32_t)IMU4_raw[13]))/2;//Z-Accel
			IMU_data[3]=(((int32_t)IMU3_raw[14]<<24|(int32_t)IMU3_raw[15]<<16|(int32_t)IMU3_raw[16]<<8|(int32_t)IMU3_raw[17])-((int32_t)IMU4_raw[14]<<24|(int32_t)IMU4_raw[15]<<16|(int32_t)IMU4_raw[16]<<8|(int32_t)IMU4_raw[17]))/2;//X-Gyro
			IMU_data[4]=(((int32_t)IMU3_raw[18]<<24|(int32_t)IMU3_raw[19]<<16|(int32_t)IMU3_raw[20]<<8|(int32_t)IMU3_raw[21])-((int32_t)IMU4_raw[18]<<24|(int32_t)IMU4_raw[19]<<16|(int32_t)IMU4_raw[20]<<8|(int32_t)IMU4_raw[21]))/2;//Y-Gyro
			IMU_data[5]=(((int32_t)IMU3_raw[22]<<24|(int32_t)IMU3_raw[23]<<16|(int32_t)IMU3_raw[0]<<8|(int32_t)IMU3_raw[1])+((int32_t)IMU4_raw[22]<<24|(int32_t)IMU4_raw[23]<<16|(int32_t)IMU4_raw[0]<<8|(int32_t)IMU4_raw[1]))/2;//Z-Gyro
		}

		else{
			if(IMU3_sum!=IMU3_sump && IMU4_sum==IMU4_sump){
				IMU_data[0]=((int32_t)IMU3_raw[2]<<24|(int32_t)IMU3_raw[3]<<16|(int32_t)IMU3_raw[4]<<8|(int32_t)IMU3_raw[5]);//X-Accel
				IMU_data[1]=((int32_t)IMU3_raw[6]<<24|(int32_t)IMU3_raw[7]<<16|(int32_t)IMU3_raw[8]<<8|(int32_t)IMU3_raw[9]);//Y-Accel
				IMU_data[2]=((int32_t)IMU3_raw[10]<<24|(int32_t)IMU3_raw[11]<<16|(int32_t)IMU3_raw[12]<<8|(int32_t)IMU3_raw[13]);//Z-Accel
				IMU_data[3]=((int32_t)IMU3_raw[14]<<24|(int32_t)IMU3_raw[15]<<16|(int32_t)IMU3_raw[16]<<8|(int32_t)IMU3_raw[17]);//X-Gyro
				IMU_data[4]=((int32_t)IMU3_raw[18]<<24|(int32_t)IMU3_raw[19]<<16|(int32_t)IMU3_raw[20]<<8|(int32_t)IMU3_raw[21]);//Y-Gyro
				IMU_data[5]=((int32_t)IMU3_raw[22]<<24|(int32_t)IMU3_raw[23]<<16|(int32_t)IMU3_raw[0]<<8|(int32_t)IMU3_raw[1]);//Z-Gyro

				//printf("%g,%d,%g\n",0.00025*((int32_t)IMU3_raw[10]<<24|(int32_t)IMU3_raw[11]<<16|(int32_t)IMU3_raw[12]<<8|(int32_t)IMU3_raw[13])/65536,k,0.00025*IMU_data_sum[2]/65536);
			}

			if(IMU4_sum!=IMU4_sump && IMU3_sum==IMU3_sump){
				IMU_data[0]=-((int32_t)IMU4_raw[2]<<24|(int32_t)IMU4_raw[3]<<16|(int32_t)IMU4_raw[4]<<8|(int32_t)IMU4_raw[5]);//X-Accel
				IMU_data[1]=-((int32_t)IMU4_raw[6]<<24|(int32_t)IMU4_raw[7]<<16|(int32_t)IMU4_raw[8]<<8|(int32_t)IMU4_raw[9]);//Y-Accel
				IMU_data[2]=((int32_t)IMU4_raw[10]<<24|(int32_t)IMU4_raw[11]<<16|(int32_t)IMU4_raw[12]<<8|(int32_t)IMU4_raw[13]);//Z-Accel
				IMU_data[3]=-((int32_t)IMU4_raw[14]<<24|(int32_t)IMU4_raw[15]<<16|(int32_t)IMU4_raw[16]<<8|(int32_t)IMU4_raw[17]);//X-Gyro
				IMU_data[4]=-((int32_t)IMU4_raw[18]<<24|(int32_t)IMU4_raw[19]<<16|(int32_t)IMU4_raw[20]<<8|(int32_t)IMU4_raw[21]);//Y-Gyro
				IMU_data[5]=((int32_t)IMU4_raw[22]<<24|(int32_t)IMU4_raw[23]<<16|(int32_t)IMU4_raw[0]<<8|(int32_t)IMU4_raw[1]);//Z-Gyro
			}

			if(IMU4_sum==IMU4_sump && IMU3_sum==IMU3_sump){
				IMU_data[0]=IMU_data[0];
				IMU_data[1]=IMU_data[1];
				IMU_data[2]=IMU_data[2];
				IMU_data[3]=IMU_data[3];
				IMU_data[4]=IMU_data[4];
				IMU_data[5]=IMU_data[5];
			}
		}

			IMU3_sump=IMU3_sum;
			IMU4_sump=IMU4_sum;

			Navcomp_send_buff[4]=(uint8_t)((NB_counter & 0xFF00)>>8);
			Navcomp_send_buff[3]=(uint8_t)(NB_counter & 0x00FF);
			NB_counter++;

			i=0;
			for(i=0;i<6;i++){
				Navcomp_send_buff[4*i+10]=(BYTE)((uint32_t)(IMU_data[i] & 0xFF000000)>>24);
				Navcomp_send_buff[4*i+9]=(BYTE)((uint32_t)(IMU_data[i] & 0x00FF0000)>>16);
				Navcomp_send_buff[4*i+8]=(BYTE)((uint32_t)(IMU_data[i] & 0x0000FF00)>>8);
				Navcomp_send_buff[4*i+7]=(BYTE)((uint32_t)(IMU_data[i] & 0x000000FF));

			}

			Navcomp_send_buff[47]=statestatus;
			//Calculating checksum
			sum=0;
			i=0;
			for(i=3;i<63;i++){
				sum =0xFF & (sum + Navcomp_send_buff[i]);
			}

			Navcomp_send_buff[63]=(uint8_t)(sum);

			//if(FiftyHzTaskFlag==1){
				i=0;
				for(i=0;i<sizeof(Navcomp_send_buff);i++){
					write(fdNavcomp,&Navcomp_send_buff[i],1);
				}

				//printf("%g\n",0.00025*IMU_data[2]/65536);

				FiftyHzTaskFlag=0;
			//}

			//



	}//While

}//Main Task



