/*
 * trig.h
 *
 *  Created on: 23 lut 2017
 *      Author: Harimaru
 */

#include <inttypes.h>

#ifndef TRIG_H_
#define TRIG_H_

int8_t check_quadrant(float angle);

float fast_sin(float a);

float fast_cos(float a);

float fast_asin(float a);

float fast_acos(float a);

float fast_atan(float a);

#endif /* TRIG_H_ */
