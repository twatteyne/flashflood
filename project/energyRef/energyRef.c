#include "msp430f1611.h"
#include "stdint.h"

/**
\brief The program test the energy consumption under different Low Power Mode.

Replace LPM0_bits by :
  LPM0_bits
  LPM1_bits
  LPM2_bits
  LPM3_bits
  LPM4_bits
  LPM5_bits
to change different Low Power Mode (LPM)
*/
int main(void) {
   WDTCTL     =  WDTPW + WDTHOLD;                // disable watchdog timer
   
   DCOCTL     =  DCO0 | DCO1 | DCO2;             // MCLK at 8MHz
   BCSCTL1    =  RSEL0 | RSEL1 | RSEL2;          // MCLK at 8MHz
   
   __bis_SR_register(GIE+LPM0_bits);             // sleep
}