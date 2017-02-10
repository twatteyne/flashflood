/**
\brief TelosB-specific definition of the "bsp_timer" bsp module.

On TelosB, we use timerA0 for the bsp_timer module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, March 2012.
*/

#include "msp430f1611.h"
#include "bsp_timer.h"
#include "board.h"

//=========================== defines =========================================

//=========================== variables =======================================

typedef struct {
   bsp_timer_cbt    overflowCb;
   bsp_timer_cbt    compareCb;
   bsp_timer_cbt    subtickCalculateCb;
   PORT_TIMER_WIDTH last_compare_value;
} bsp_timer_vars_t;

bsp_timer_vars_t bsp_timer_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================

/**
\brief Initialize this module.

This functions starts the timer, i.e. the counter increments, but doesn't set
any compare registers, so no interrupt will fire.
*/
void bsp_timer_init() {
    // clear local variables
    memset(&bsp_timer_vars,0,sizeof(bsp_timer_vars_t));
}

void bsp_timer_setOverflowCb(bsp_timer_cbt cb) {
    bsp_timer_vars.overflowCb     = cb;
}

void bsp_timer_setCompareCb(bsp_timer_cbt cb) {
    bsp_timer_vars.compareCb      = cb;
}

void bsp_timer_setSubtickCalculateCb(bsp_timer_cbt cb) {
    bsp_timer_vars.subtickCalculateCb = cb;
}

void bsp_timer_start(PORT_TIMER_WIDTH period){
    // radio's SFD pin connected to P4.1
    P4DIR   &= ~0x02; // input
    P4SEL   |=  0x02; // in CCI1a/B mode
    
    // set CCRA0 registers
    TACCR0   =  period-1;
    
    // CCR1 in compare mode (disabled for now)
    TACCTL1  =  0;
    TACCR1   =  0;
    
    // CCR2 in compare mode (disabled for now)
    TACCTL2  =  0;
    TACCR2   =  0;
    
    //start TimerA
    TACTL    =  TAIE+TACLR;    // interrupt when counter resets
    TACTL   |=  MC_1+TASSEL_1; // up mode, from ACLK
}

PORT_TIMER_WIDTH bsp_timer_get_currentValue() {
   return TAR;
}

void bsp_timer_setPeriod(PORT_TIMER_WIDTH period) {
    TACCR0   =  period;
}

PORT_TIMER_WIDTH bsp_timer_getPeriod() {
    return TACCR0;
}

//===== compare

void bsp_timer_schedule_subTickCalc(PORT_TIMER_WIDTH offset){
   // offset when to fire
   TACCR1   =  offset;
   
   // enable compare interrupt (this also cancels any pending interrupts)
   TACCTL1  =  CCIE;
}

void bsp_timer_subTickCalc_cancel() {
   // reset compare value (also resets interrupt flag)
   TACCR1   =  0;
   
   // disable compare interrupt
   TACCTL1 &= ~CCIE;
}

void bsp_timer_schedule(PORT_TIMER_WIDTH offset) {
   // offset when to fire
   TACCR2   =  offset;
   
   // enable compare interrupt (this also cancels any pending interrupts)
   TACCTL2  =  CCIE;
}

void bsp_timer_cancel() {
   // reset compare value (also resets interrupt flag)
   TACCR2   =  0;
   
   // disable compare interrupt
   TACCTL2 &= ~CCIE;
}

void bsp_timer_reset(){
    TAR     = 0;
}

//=========================== private =========================================

//=========================== interrup handlers ===============================

kick_scheduler_t bsp_timer_isr() {
   PORT_RADIOTIMER_WIDTH taiv_local;
   
   // reading TAIV returns the value of the highest pending interrupt flag
   // and automatically resets that flag. We therefore copy its value to the
   // tbiv_local local variable exactly once. If there is more than one 
   // interrupt pending, we will reenter this function after having just left
   // it.
   taiv_local = TAIV;
   
   switch (taiv_local) {
      case 0x0002: // CCR1 fires
         if (bsp_timer_vars.subtickCalculateCb!=NULL) {
            bsp_timer_vars.subtickCalculateCb();
            // kick the OS
            return KICK_SCHEDULER;
         }
         break;
      case 0x0004: // CCR2 fires
         if (bsp_timer_vars.compareCb!=NULL) {
            bsp_timer_vars.compareCb();
            // kick the OS
            return KICK_SCHEDULER;
         }
         break;
      case 0x000a: // timer overflow
         if (bsp_timer_vars.overflowCb!=NULL) {
            bsp_timer_vars.overflowCb();
            // kick the OS
            return KICK_SCHEDULER;
         }
         break;
   }
   return DO_NOT_KICK_SCHEDULER;
}

#pragma vector = TIMERA1_VECTOR
__interrupt void TIMERA1_ISR (void) {
   PORT_RADIOTIMER_WIDTH taiv_local;
   taiv_local = TAIV;
   
   if (taiv_local==0x0002) {
      P6OUT |=  0x01;
      bsp_timer_vars.subtickCalculateCb();
   } else {
      if (taiv_local==0x0004) {
          P6OUT |=  0x01;
          bsp_timer_vars.compareCb();
      } else {
          if (taiv_local==0x000a){
              if (bsp_timer_vars.overflowCb!=NULL) {
                  P6OUT |=  0x01;
                  bsp_timer_vars.overflowCb();
              }
          } else {
          }
      }
   }
   P6OUT &= ~0x01;
   __bic_SR_register_on_exit(CPUOFF);
}