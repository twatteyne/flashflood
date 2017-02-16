#include "msp430f1611.h"
#include "stdint.h"

/**
\brief The program test the energy consumption under LPM4.
*/
int main(void) {
   WDTCTL     =  WDTPW + WDTHOLD;                // disable watchdog timer
   
   DCOCTL     =  DCO0 | DCO1 | DCO2;             // MCLK at 8MHz
   BCSCTL1    =  RSEL0 | RSEL1 | RSEL2;          // MCLK at 8MHz
   
   __bis_SR_register(GIE+LPM4_bits);             // sleep
}