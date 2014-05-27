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

float ReadLaser(int Laserserial){

	uint8_t i=0;
	static char in_buff[30];
	int laser_timeout=100;

	i=0;
	while(1)
	{
		laser_timeout=ReadWithTimeout(Laserserial,&in_buff[i],1,1);//Reading data from laser rangefinder
		if(laser_timeout==0 || laser_timeout==-1)
			break;
		if(in_buff[i]==10){
			return(atof(in_buff));
			break;
		}

		i++;
	}

	return 0;
	//laser_in_buff_length=i;

	//printf("laser range=%g\n",laser_range);
	//memset(in_buff,0,sizeof(in_buff));
}


