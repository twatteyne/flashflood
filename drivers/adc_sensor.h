#ifndef __ADC_SENSOR_H__
#define __ADC_SENSOR_H__

#include "stdint.h"
#include "msp430f1611.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== module variables ================================

//=========================== prototypes ======================================

void     adc_init(void);
uint16_t adc_read_light(void);

#endif // __ADC_SENSOR_H__
