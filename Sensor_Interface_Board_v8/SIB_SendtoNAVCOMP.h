/*
 * SIB_SendtoNAVCOMP.h
 *
 *  Created on: May 11, 2014
 *      Author: Tanmay
 */

#ifndef SIB_SENDTONAVCOMP_H_
#define SIB_SENDTONAVCOMP_H_

#include <basictypes.h>

#ifdef __cplusplus
extern "C"
{
#endif

void SendtoNAVCOMP(char* Databuff,uint16_t* ADCbuff,char* time_ms,uint16_t NBcounter,float Range_Laser,unsigned char* Radio_buff,int16_t GimalPAN,int NAVserial,int debug_Serial,int Databuff_size,uint8_t Ant_config);


#ifdef __cplusplus
}
#endif


#endif /* SIB_SENDTONAVCOMP_H_ */
