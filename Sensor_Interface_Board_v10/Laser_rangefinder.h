/*
 * Laser_rangefinder.h
 *
 *  Created on: May 11, 2014
 *      Author: Tanmay
 */
#include <basictypes.h>

#ifndef LASER_RANGEFINDER_H_
#define LASER_RANGEFINDER_H_



#ifdef __cplusplus
extern "C"
{
#endif

int16_t StartUpLaserScan(int Laseserial);
float ReadLaser(int Laserserial);


#ifdef __cplusplus
}
#endif


#endif /* LASER_RANGEFINDER_H_ */
