/**
\brief TelosB-specific definition of the "radiotimer" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#include "msp430f1611.h"
#include "radiotimer.h"
#include "leds.h"

//=========================== variables =======================================

typedef struct {
   radiotimer_compare_cbt    compareCb;
   radiotimer_capture_cbt    startFrameCb;
   radiotimer_capture_cbt    endFrameCb;
   uint8_t                   f_SFDreceived;
} radiotimer_vars_t;

radiotimer_vars_t radiotimer_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================

//===== admin

void radiotimer_init() {
   // clear local variables
   memset(&radiotimer_vars,0,sizeof(radiotimer_vars_t));
   
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

void radiotimer_setCompareCb(radiotimer_compare_cbt cb) {
   radiotimer_vars.compareCb      = cb;
}

void radiotimer_setStartFrameCb(radiotimer_capture_cbt cb) {
   radiotimer_vars.startFrameCb   = cb;
}

void radiotimer_setEndFrameCb(radiotimer_capture_cbt cb) {
   radiotimer_vars.endFrameCb     = cb;
}

//===== compare

void radiotimer_scheduleIn(PORT_TIMER_WIDTH offset) {
   // reset timer
   TBR      = 0;
   
   // offset when to fire
   TBCCR2   =  offset;
   
   // enable compare interrupt (this also cancels any pending interrupts)
   TBCCTL2  =  CCIE;
}

void radiotimer_cancel() {
   // reset compare value (also resets interrupt flag)
   TBCCR2   =  0;
   
   // disable compare interrupt
   TBCCTL2 &= ~CCIE;
}

void radiotimer_reset(){
    // reset timer
    TBR     = 0;
    // reset compare value (also resets interrupt flag)
    TBCCR2  = 0;
    // disable compare interrupt
    TBCCTL2 = 0;
}

//===== direct access

PORT_RADIOTIMER_WIDTH radiotimer_getValue() {
   return TBR;
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================

/**
\brief TimerB CCR1-6 interrupt service routine
*/
kick_scheduler_t radiotimer_isr() {
   PORT_RADIOTIMER_WIDTH tbiv_local;
   
   // reading TBIV returns the value of the highest pending interrupt flag
   // and automatically resets that flag. We therefore copy its value to the
   // tbiv_local local variable exactly once. If there is more than one 
   // interrupt pending, we will reenter this function after having just left
   // it.
   tbiv_local = TBIV;
   
   switch (tbiv_local) {
      case 0x0002: // CCR1 fires
         if (TBCCTL1 & CCI) {
            // SFD pin is high: this was the start of a frame
            if (radiotimer_vars.startFrameCb!=NULL) {
               radiotimer_vars.startFrameCb(TAR);
               radiotimer_vars.f_SFDreceived = 1;
               // kick the OS
               return KICK_SCHEDULER;
            }
         } else {
            // SFD pin is low: this was the end of a frame
            if (radiotimer_vars.endFrameCb!=NULL) {
               if (radiotimer_vars.f_SFDreceived == 1) {
                  radiotimer_vars.endFrameCb(TAR);
                  radiotimer_vars.f_SFDreceived = 0;
               }
               TBCCTL1 &= ~COV;
               TBCCTL1 &= ~CCIFG;
               // kick the OS
               return KICK_SCHEDULER;
            }
         }
         break;
      case 0x0004: // CCR2 fires
          if (radiotimer_vars.compareCb!=NULL){
              radiotimer_vars.compareCb();
          }
          return KICK_SCHEDULER;
         break;
      case 0x0006: // CCR3 fires
         break;
      case 0x0008: // CCR4 fires
         break;
      case 0x000a: // CCR5 fires
         break;
      case 0x000c: // CCR6 fires
         break;
      case 0x000e: // timer overflow
         break;
   }
   return DO_NOT_KICK_SCHEDULER;
}

#pragma vector = TIMERB1_VECTOR
__interrupt void TIMERB1_ISR (void) {
   PORT_RADIOTIMER_WIDTH tbiv_local;
   tbiv_local = TBIV;
   if (tbiv_local==0x0002){
        if (TBCCTL1 & CCI) {
             P6OUT |=  0x01;
             radiotimer_vars.startFrameCb(TAR);
             radiotimer_vars.f_SFDreceived = 1;
        } else {
             if (radiotimer_vars.f_SFDreceived == 1) {
                 P6OUT |=  0x01;
                 radiotimer_vars.endFrameCb(TAR);
                 radiotimer_vars.f_SFDreceived = 0;
             }
             TBCCTL1 &= ~COV;
             TBCCTL1 &= ~CCIFG;
        }
   } else {
        if (tbiv_local==0x0004){
            P6OUT |=  0x01;
            radiotimer_vars.compareCb();
        } else {
        }
   }
   P6OUT &= ~0x01;
   __bic_SR_register_on_exit(CPUOFF);
}
