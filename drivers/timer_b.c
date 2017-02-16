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
   timer_b_compare_cbt    compareCb;
   timer_b_capture_cbt    startFrameCb;
   timer_b_capture_cbt    endFrameCb;
   uint8_t                f_SFDreceived;
   uint16_t               offset;
   uint8_t                packetTobeSent;
} timer_b_vars_t;

timer_b_vars_t timer_b_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================

//===== admin

void timer_b_init() {
    // clear local variables
    memset(&timer_b_vars,0,sizeof(timer_b_vars_t));
    
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

void timer_b_setCompareCb(timer_b_compare_cbt cb) {
    timer_b_vars.compareCb      = cb;
}

void timer_b_setStartFrameCb(timer_b_capture_cbt cb) {
    timer_b_vars.startFrameCb   = cb;
}

void timer_b_setEndFrameCb(timer_b_capture_cbt cb) {
    timer_b_vars.endFrameCb     = cb;
}

void timer_b_setOffset(uint16_t offset){
    timer_b_vars.offset = offset;
}

void timer_b_setPacketTobeSent(){
    timer_b_vars.packetTobeSent = 1;
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
#pragma vector = TIMERB1_VECTOR
__interrupt void TIMERB1_ISR (void) {
   uint16_t tbiv_local;
   tbiv_local = TBIV;
#ifdef PIN_DEBUG
   P6OUT |=  0x40;
#endif   
   if (tbiv_local==0x0004){
#ifdef PIN_DEBUG
        P6OUT |=  0x80;
#endif
        // send TXON strobe
        
        // pull high after timer experid.
        P4OUT  &= ~0x04;
        // write next byte to TX buffer
        U0TXBUF  = CC2420_STXON;
        // busy wait on the interrupt flag
        while ((IFG1 & URXIFG0)==0);
        // clear the interrupt flag
        IFG1    &= ~URXIFG0;
        // pull highs
        P4OUT   |=  0x04;

        TBCCR2   =  0;
        TBCCTL2 &= ~CCIE;
#ifdef PIN_DEBUG
        P6OUT &= ~0x80;
#endif
   } else {
       if (tbiv_local==0x0002){
            if (TBCCTL1 & CCI) {
                 timer_b_vars.f_SFDreceived = 1;
            } else {
                 if (timer_b_vars.f_SFDreceived == 1) {
                     timer_b_vars.endFrameCb(TBCCR1);
                     timer_b_vars.f_SFDreceived = 0;
                 }
                 TBCCTL1 &= ~COV;
                 TBCCTL1 &= ~CCIFG;
            }
       } else {
            if (tbiv_local==0x000e){
                //overflow
            }
       }
   }
#ifdef PIN_DEBUG
   P6OUT &= ~0x40;
#endif
   __bic_SR_register_on_exit(CPUOFF);
}
