/**
\brief Driver which reads the value of an ADC channel.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, March 2012.
*/

#ifndef __ADC_SENSOR_H
#define __ADC_SENSOR_H

#include <stdint.h>

#include "msp430f1611.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== module variables ================================

//=========================== prototypes ======================================

void     adc_init(void);
uint16_t adc_read_light(void);

#endif
