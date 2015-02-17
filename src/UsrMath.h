/*
 * Map.h
 *
 *  Created on: Jan 28, 2015
 *      Author: Developer
 */

#ifndef FRCDEVELOPMENT2015_SRC_USRMATH_H_
#define FRCDEVELOPMENT2015_SRC_USRMATH_H_

long Map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif /* FRCDEVELOPMENT2015_SRC_USRMATH_H_ */
