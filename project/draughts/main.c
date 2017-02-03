//=========================== include =========================================

#include "msp430f1611.h"
#include "stdint.h"

//=========================== main ============================================

int main(void){
    
    // disable watchdog timer
    WDTCTL     =  WDTPW + WDTHOLD;
    
    // setup clock speed
    DCOCTL    |=  DCO0 | DCO1 | DCO2;             // MCLK at ~8MHz
    BCSCTL1   |=  RSEL0 | RSEL1 | RSEL2;          // MCLK at ~8MHz
                                                 // by default, ACLK from 32kHz XTAL which is running
    // initialize pins
    P4DIR     |=  0x20;                           // [P4.5] radio VREG:  output
    P4DIR     |=  0x40;                           // [P4.6] radio reset: output
 
    // initialize leds
    P5DIR     |=  0x70;                           // P5DIR = 0bx111xxxx for LEDs
    P5OUT     |=  0x70;                           // P5OUT = 0bx111xxxx, all LEDs off
    P5OUT     &=  0x8F;                           // P5OUT = 0bx111xxxx, all LEDs on
    
    // initialize uart
     
    // initialize spi
    
    // initialize timer
    
    TACCTL0    =  CCIE;                           // capture/compare interrupt enable
    TACCR0     =  16000;                          // 16000@32kHz ~ 500ms
    TACTL      =  MC_1+TASSEL_1;                  // up mode, using ACLK
    
    __bis_SR_register(GIE+LPM3_bits);             // sleep, but leave ACLK on
}