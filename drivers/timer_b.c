/**
\brief TelosB-specific definition of the "timer_b" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#include "msp430f1611.h"
#include "timer_b.h"
#include "string.h"
#include "cc2420.h"

//=========================== variables =======================================

typedef struct {
   timer_b_capture_cbt    endFrameCb;
   uint8_t                f_SFDreceived;
} timer_b_vars_t;

timer_b_vars_t timer_b_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================

//===== admin

void timer_b_init() {
    // clear local variables
    memset(&timer_b_vars,0,sizeof(timer_b_vars_t));
    
    // radio's SFD pin connected to P4.1 (timer B capture CCR1)
    P4DIR   &= ~0x02; // input
    P4SEL   |=  0x02; // in CCI1a/B mode
    
    // radio FIFOP pin connected to P1.0
    P1DIR   &= ~0x01; // input
    
    // CCR1 in capture mode
    TBCCTL1  =  CM_3+SCS+CAP+CCIE;
    TBCCR1   =  0;
    
    // CCR2 in compare mode (disabled for now)
    TBCCTL2  =  0;
    TBCCR2   =  0;
    
    // start counting
    TBCTL    =  TBIE+TBCLR;                       // interrupt when counter resets
    TBCTL   |=  MC_2+TBSSEL_2;                    // continue mode, clocked from SMCLK
}

void timer_b_setEndFrameCb(timer_b_capture_cbt cb) {
    timer_b_vars.endFrameCb     = cb;
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================

#pragma vector = TIMERB1_VECTOR
__interrupt void TIMERB1_ISR (void) {
   uint16_t tbiv_local;
   uint16_t timestamp_timerA = TAR;
   tbiv_local = TBIV;
#ifdef ENABLE_DEBUGPINS
   P6OUT |=  0x40; // P6.6 [timerBisr]
#endif   
   if (tbiv_local==0x0004){
        // CCR2 compare fired
       
        // send TXON strobe
        
        P4OUT     &= ~0x04;
        U0TXBUF    =  CC2420_STXON;
        while ((IFG1 & URXIFG0)==0);
        IFG1      &= ~URXIFG0;
        P4OUT     |=  0x04;

        TBCCR2   =  0;
        TBCCTL2 &= ~CCIE;
        
   } else {
       if (tbiv_local==0x0002){
            // CCR1 capture triggered (SFD pin toggled)
           
            if (TBCCTL1 & CCI) {
                 // SFD pin went high
                
#ifdef ENABLE_DEBUGPINS
                 P3OUT |=  0x20; // P3.5 [sfd]
#endif
                 
                 timer_b_vars.f_SFDreceived = 1;
                 
            } else {
                 // SFD pin went low
                
#ifdef ENABLE_DEBUGPINS
                 P3OUT &= ~0x20; // P3.5 [sfd]
#endif
                 
                 if (timer_b_vars.f_SFDreceived==1) {
                     //if (P1IN & 0x01) {
                         timer_b_vars.endFrameCb(timestamp_timerA,TBCCR1);
                     //}
                     timer_b_vars.f_SFDreceived = 0;
                 }
                 TBCCTL1 &= ~COV;
                 TBCCTL1 &= ~CCIFG;
            }
       } else {
            if (tbiv_local==0x000e){
                // overflow
            }
       }
   }
   
#ifdef ENABLE_DEBUGPINS
   P6OUT &= ~0x40; // P6.6 [timerBisr]
#endif
   __bic_SR_register_on_exit(CPUOFF);
}
