#include "msp430f1611.h"
#include "stdint.h"
#include "adc_sensor.h"

/**
\brief The program test the energy consumption under LPM4.
*/
int main(void) {
    uint16_t lightValue;
    
    // disable watchdog timer
    WDTCTL     =  WDTPW + WDTHOLD;
   
    // setup clock speed
    DCOCTL     =  DCO0 | DCO1 | DCO2;             // MCLK at 8MHz
    BCSCTL1    =  RSEL0 | RSEL1 | RSEL2;          // MCLK at 8MHz
    
    adc_sensor_init();
    while (1) {
        lightValue = adc_sens_read_total_solar();
        lightValue = lightValue; // to avoid that "lightValue" gets optimized away
    }
    
    //__bis_SR_register(GIE+LPM4_bits);              // sleep
}