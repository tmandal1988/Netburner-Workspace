/*
 * SIB_SendtoNAVCOMP.cpp
 *
 *  Created on: May 11, 2014
 *      Author: Tanmay
 */


#include "SIB_SendtoNAVCOMP.h"
#include <basictypes.h>
#include <serial.h>
#include <iosys.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

void SendtoNAVCOMP(char* Databuff,uint16_t* ADCbuff,char* time_ms,uint16_t NBcounter,float Range_Laser,unsigned char* Radio_buff,int16_t GimbalPAN,int NAVserial,int debug_Serial,int Databuff_size,uint8_t Ant_config){

	uint16_t checksum=0;
	uint8_t i=0;
	//send data to the computer
	/*******packing 16 bit Netburner counter***********/
	Databuff[4]=(uint8_t)((NBcounter & 0xFF00)>>8);
	Databuff[3]=(uint8_t)(NBcounter & 0x00FF);

	/*******packing 16 bit ms timer****************/
	Databuff[6]=time_ms[0];
	Databuff[5]=time_ms[1];

	//printf("\n");
	Databuff[8]=(uint8_t)((ADCbuff[0] & 0xFF00)>>8);
	Databuff[7]=(uint8_t)(ADCbuff[0] & 0x00FF);

	Databuff[10]=(uint8_t)((ADCbuff[1] & 0xFF00)>>8);
	Databuff[9]=(uint8_t)(ADCbuff[1] & 0x00FF);

	Databuff[12]=(uint8_t)((ADCbuff[2] & 0xFF00)>>8);
	Databuff[11]=(uint8_t)(ADCbuff[2] & 0x00FF);

	Databuff[14]=(uint8_t)((ADCbuff[3] & 0xFF00)>>8);
	Databuff[13]=(uint8_t)(ADCbuff[3] & 0x00FF);

	Databuff[16]=(uint8_t)((ADCbuff[4] & 0xFF00)>>8);
	Databuff[15]=(uint8_t)(ADCbuff[4] & 0x00FF);

	Databuff[18]=(uint8_t)((ADCbuff[5] & 0xFF00)>>8);
	Databuff[17]=(uint8_t)(ADCbuff[5] & 0x00FF);

	Databuff[20]=(uint8_t)((ADCbuff[6] & 0xFF00)>>8);
	Databuff[19]=(uint8_t)(ADCbuff[6] & 0x00FF);

	Databuff[22]=(uint8_t)((ADCbuff[7] & 0xFF00)>>8);
	Databuff[21]=(uint8_t)(ADCbuff[7] & 0x00FF);

	/***********Packing laser rangefinder data****************/


	Databuff[23]=(uint8_t)((uint16_t)(Range_Laser*6553.5)& 0x00FF);
	Databuff[24]=(uint8_t)(((uint16_t)(Range_Laser*6553.5)& 0xFF00)>>8);

	/**************packing ranging radio solution status**********************/
	//Databuff[26]=Radio_buff[6];

	/*********packing radio filtered range standard deviation uint16 (mm units)*******/
	Databuff[27]=Radio_buff[5];
	Databuff[28]=Radio_buff[4];


	/**********packing radio ranging data*********************/
	Databuff[29]=Radio_buff[3];
	Databuff[30]=Radio_buff[2];
	Databuff[31]=Radio_buff[1];
	Databuff[32]=Radio_buff[0];

	/*********packing radio antenna configuration*************/
	Databuff[33]=Ant_config;
	/************packing stepper motor angle from encoder**********/
	Databuff[35] = (uint8_t)((GimbalPAN & 0xFF00)>>8);
	Databuff[34] = (uint8_t)(GimbalPAN & 0x00FF);



	checksum=0;
	i=0;
	for(i=3;i<47;i++){
		checksum +=Databuff[i];
	}

	Databuff[47]=(uint8_t)(checksum % 256);

	/************sending data out********************************/
	for (i=0;i<Databuff_size;i++){
		write(NAVserial,&Databuff[i],1);//goes to vision computer
		//printf("Hi\n");
		write(debug_Serial,&Databuff[i],1);//goes to mission computer
	}

}
