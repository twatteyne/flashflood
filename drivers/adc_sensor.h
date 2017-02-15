#ifndef __ADC_SENSOR_H__
#define __ADC_SENSOR_H__

#include "stdint.h"
#include "msp430f1611.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== module variables ================================

//=========================== prototypes ======================================

void adc_sensor_init(void);
uint16_t adc_sens_read_photosynthetic(void);
float adc_sens_convert_photosynthetic(uint16_t cputemp);
uint16_t adc_sens_read_total_solar(void);
float adc_sens_convert_total_solar(uint16_t cputemp);

#endif // __ADC_SENSOR_H__
