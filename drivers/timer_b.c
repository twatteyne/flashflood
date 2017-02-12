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
   cc2420_status_t statusByte;
   tbiv_local = TBIV;
   
   P6OUT |=  0x40;
   // for calculating subticks, cancel it later if this is not overflow
   TACCR1  =  TAR+TIMER_A_SUBTICK;
   TACCTL1 =  CCIE;
   
   if (tbiv_local==0x0004){
        P6OUT |=  0x80;
        // send out data
        cc2420_spiStrobe(CC2420_STXON, &statusByte);
        P6OUT &= ~0x80;
        TACCTL1  =  0;
        TACCR1  &= ~CCIE;
   } else {
       if (tbiv_local==0x0002){
            if (TBCCTL1 & CCI) {
                 timer_b_vars.startFrameCb(TBCCR1);
                 timer_b_vars.f_SFDreceived = 1;
            } else {
                 if (timer_b_vars.f_SFDreceived == 1) {
                     timer_b_vars.endFrameCb(TBCCR1);
                     timer_b_vars.f_SFDreceived = 0;
                 }
                 TBCCTL1 &= ~COV;
                 TBCCTL1 &= ~CCIFG;
            }
            TACCTL1  =  0;
            TACCR1  &= ~CCIE;
       } else {
            if (tbiv_local==0x000e){
                //overflow, don't cancel CCR1 on timer A
                P2OUT |= 0x40; 
            } else {
                TACCTL1  =  0;
                TACCR1  &= ~CCIE;
            }
       }
   }
   P6OUT &= ~0x40;
   __bic_SR_register_on_exit(CPUOFF);
}
