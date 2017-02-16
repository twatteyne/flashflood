#include "msp430f1611.h"
#include "stdint.h"
#include "radio.h"

/**
\brief The program test the energy consumption under LPM3 when radio is in rx.
*/
int main(void) {
   uint16_t   delay;
   WDTCTL     =  WDTPW + WDTHOLD;                // disable watchdog timer
   
   DCOCTL     =  DCO0 | DCO1 | DCO2;             // MCLK at 8MHz
   BCSCTL1    =  RSEL0 | RSEL1 | RSEL2;          // MCLK at 8MHz
   
   /**** initialize radio */
    // initialize pins
    P4DIR     |=  0x20;                           // [P4.5] radio VREG:  output
    P4DIR     |=  0x40;                           // [P4.6] radio reset: output
    // set radio VREG pin high
    P4OUT |=  0x20;
    for (delay=0xffff;delay>0;delay--);           // max. VREG start-up time is 0.6ms
    // set radio RESET pin low
    P4OUT &= ~0x40; 
    for (delay=0xffff;delay>0;delay--);
    // set radio RESET pin high
    P4OUT |=  0x40;
    for (delay=0xffff;delay>0;delay--);
    
    // turn on radio osc
    radio_rfOn();
    // change to rx state
    radio_rxNow();
   
   __bis_SR_register(GIE+LPM3_bits);             // sleep
}