/*
 * Laser_rangefinder.cpp
 *
 *  Created on: May 11, 2014
 *      Author: Tanmay
 *
 */

#include "Laser_rangefinder.h"
#include <basictypes.h>
#include <serial.h>
#include <iosys.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sim.h>
#include <SIB_initPINS.h>
#include <ucosmcfc.h>
#include <ucos.h> //OS macros



int16_t StartUpLaserScan(int Laserserial){

	uint8_t C_NUM=19,val_sol1=0,val_sol2=0,k=0;
	uint8_t Laser_Num=20;
	int16_t StartPan=0;
	int i=0,j=0;
	uint16_t Pulse = 12287,pwmr_comp=0;
	double dYawS=0;
	double sum=0,avg_dist=0,min_dist=9999,min_angle=0;
	double c_dist[19]={0},c_ang[19]={0},f_dist[11]={0},f_ang[11]={0};

	dYawS=-180;
	Pulse=12287-dYawS*20.51;

	if(Pulse<8594 || Pulse==8594)
		sim1.mcpwm.sm[1].val[5]=8594;
	if(Pulse>15980 || Pulse==15980)
		sim1.mcpwm.sm[1].val[5]=15980;
	else
		sim1.mcpwm.sm[1].val[5]=Pulse;//PAN control

	pwmr_comp=sim1.mcpwm.mcr;
	sim1.mcpwm.mcr |=LDOK;

	while(i<C_NUM){

		dYawS=-180+i*20;
		Pulse=12287-dYawS*20.51;

		if(Pulse<8594 || Pulse==8594)
			sim1.mcpwm.sm[1].val[5]=8594;
		if(Pulse>15980 || Pulse==15980)
			sim1.mcpwm.sm[1].val[5]=15980;
		else
			sim1.mcpwm.sm[1].val[5]=Pulse;//PAN control

		pwmr_comp=sim1.mcpwm.mcr;
		sim1.mcpwm.mcr |=LDOK;

		sum=0;
		j=0;
		while(j<Laser_Num){
			sum+=ReadLaser(Laserserial);
			j++;
		}//While loop to read Laser_Num number of data

		avg_dist=sum/Laser_Num;
		if(avg_dist>1.8 || avg_dist==1.8)
			c_dist[i]=999;
		else
			c_dist[i]=avg_dist;

		//printf("%g\n",c_dist[i]);
		c_ang[i]=dYawS;
		i++;

	}//Coarse While Loop

	//Calculating number of valid solutions
	for(i=0;i<C_NUM;i++){
		if(i<8 && c_dist[i]!=999 || i==8)
			val_sol1++;
		if(i>8 && c_dist[i]!=999 && i!=8)
			val_sol2++;

	}//valid solutions for loop

	//printf("valsol1=%d,valsol2=%d\n",val_sol1,val_sol2);

	if(val_sol1 > val_sol2){
		for(i=0;i<8;i++){
			if(c_dist[i]<min_dist || c_dist[i]==min_dist){
				min_dist=c_dist[i];
				min_angle=c_ang[i];
			}
		}//first half min
	}
	else{
		for(i=8;i<C_NUM;i++){
			if(c_dist[i]<min_dist || c_dist[i]==min_dist){
				min_dist=c_dist[i];
				min_angle=c_ang[i];
			}
		}//second half min
	}

	//printf("%g\n",min_angle);

	k=0;
	i=-5;
	while(i<6){
		dYawS=min_angle+i*5;
		Pulse=12287-dYawS*20.51;
		if(Pulse<8594 || Pulse==8594)
			sim1.mcpwm.sm[1].val[5]=8594;
		if(Pulse>15980 || Pulse==15980)
			sim1.mcpwm.sm[1].val[5]=15980;
		else
			sim1.mcpwm.sm[1].val[5]=Pulse;//PAN control

		pwmr_comp=sim1.mcpwm.mcr;
		sim1.mcpwm.mcr |=LDOK;

		sum=0;
		j=0;
		while(j<Laser_Num){
			sum+=ReadLaser(Laserserial);
			j++;
		}//While loop to read Laser_Num number of data

		avg_dist=sum/Laser_Num;
		if(avg_dist>1.8 || avg_dist==1.8)
			f_dist[k]=999;
		else
			f_dist[k]=avg_dist;

		f_ang[k]=dYawS;
		k++;
		i++;

	}//finer search

	min_dist=9999;
	i=0;
	for(i=0;i<11;i++){
		if(f_dist[i]<min_dist || f_dist[i]==min_dist){
			min_dist=f_dist[i];
			min_angle=f_ang[i];
		}

	}

	dYawS=min_angle;
	Pulse=12287-dYawS*20.51;
	if(Pulse<8594 || Pulse==8594)
		sim1.mcpwm.sm[1].val[5]=8594;
	if(Pulse>15980 || Pulse==15980)
		sim1.mcpwm.sm[1].val[5]=15980;
	else
		sim1.mcpwm.sm[1].val[5]=Pulse;//PAN control

	pwmr_comp=sim1.mcpwm.mcr;
	sim1.mcpwm.mcr |=LDOK;

	StartPan=(int16_t)(min_angle*10);


	return StartPan;
}///StartUpLaserScan function

float ReadLaser(int Laserserial){

	uint8_t i=0;
	char in_buff[10]={0};
	int laser_timeout=100;
	i=0;

	while(1)
	{
		laser_timeout=ReadWithTimeout(Laserserial,&in_buff[i],1,4);//Reading data from laser rangefinder

		if(laser_timeout==0 || laser_timeout==-1)
			return 0;

		if(in_buff[i]==10){
			//printf("%s\n",in_buff);
			return(atof(in_buff));
		}

		i++;
	}

}


