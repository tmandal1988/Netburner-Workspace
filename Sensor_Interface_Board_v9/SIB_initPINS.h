/*
 * SIB_initPINS.h
 *
 *  Created on: May 11, 2014
 *      Author:Tanmay
 */

#ifndef SIB_INITPINS_H_
#define SIB_INITPINS_H_

#include <basictypes.h>

//PWM Constants
#define CR2 0xE000 //Debug enabled, Wait enabled, output A&B are independent,
#define CR1 0x0444 //400 Hz Full cycle reloaded enabled, PWN clock fclk/16, Load mode selected immediatly after LDOK being set.
//#define CR1 0x0474 //50 Hz
#define OCR	0x0000 //10-8: output of A&B&x not inverted, 5-4 3-2 1-0: output of A&B&x forced to 0 prior to output polarity control
#define DISMAP 0x0000 //Fault disable mapping register. PWM fault pin has no effect on x B and A
#define OUTEN 0x0FF0  //Output enable register. 11-8 7-4 3-0: A B enabled, x not enabled.
#define LDOK 0x000F   //Master control register. 15-12: ingored by independent mode; 3-0 Load OK,
#define P_RUN 0x0F00  //PWM generator enabled.
#define P_INIT 0
#define P_MAX 19531//400 Hz
#define P_MAX2 15625 //400 Hz 80% 2000 usec
#define P_START 0
#define P_END 11719 //400 Hz 7812/19531=0.3999 1000 usec
#define P_FACTOR400 7812500 //125 MHz/16---->19531=400 Hz PWM  ?Purpose unknown
#define T_LIMIT 7800 //400 Hz  purpose unknown

#ifdef __cplusplus
extern "C"
{
#endif

void initPINS();


#ifdef __cplusplus
}
#endif


#endif /* SIB_INITPINS_H_ */
