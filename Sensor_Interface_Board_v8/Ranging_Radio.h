/*
 * Ranging_Radio.h
 *
 *  Created on: May 11, 2014
 *      Author:Tanmay
 */

#ifndef RANGING_RADIO_H_
#define RANGING_RADIO_H_

#ifdef __cplusplus
extern "C"
{
#endif


unsigned char* ReadRadio(char* CommandBuff,int radioserial,int buff_size);
unsigned short crc16(unsigned char *buf, int len);



#ifdef __cplusplus
}
#endif


#endif /* RANGING_RADIO_H_ */
