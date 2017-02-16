#include "msp430f1611.h"
#include "timer_a.h"
#include "string.h"

//=========================== defines =========================================

//=========================== variables =======================================

typedef struct {
   timer_a_cbt    overflowCb;
   timer_a_cbt    compareCCR2Cb;
   timer_a_capture_cbt    CompareCCR1andReturnTBRcb;
   uint16_t last_compare_value;
} timer_a_vars_t;

timer_a_vars_t timer_a_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================

void timer_a_init() {
    // clear local variables
    memset(&timer_a_vars,0,sizeof(timer_a_vars_t));
    
    // set CCRA0 registers
    TACCR0   =  0;
    
    // CCR1 in compare mode (disabled for now)
    TACCTL1  =  0;
    TACCR1   =  0;
    
    // CCR2 in compare mode (disabled for now)
    TACCTL2  =  0;
    TACCR2   =  0;
    
    //start TimerA
    TACTL    =  TAIE+TACLR;    // interrupt when counter resets
    TACTL   |=  MC_2+TASSEL_1; // continue mode, from ACLK
}

void timer_a_setOverflowCb(timer_a_cbt cb) {
    timer_a_vars.overflowCb           = cb;
}

void timer_a_setCompareCCR2Cb(timer_a_cbt cb) {
    timer_a_vars.compareCCR2Cb         = cb;
}

void timer_a_setCompareCCR1andReturnTBRcb(timer_a_capture_cbt cb) {
    timer_a_vars.CompareCCR1andReturnTBRcb    = cb;
}

//=========================== interrup handlers ===============================

#pragma vector = TIMERA1_VECTOR
__interrupt void TIMERA1_ISR (void) {
   uint16_t taiv_local;
   uint16_t timestamp;
   timestamp  = TBR;
   taiv_local = TAIV;
#ifdef PIN_DEBUG
      P3OUT |=  0x10;
#endif
   if (taiv_local==0x0002) {
       // CCR1 compare happeded
       
#ifdef PIN_DEBUG
       P2OUT ^= 0x40;
#endif
       timer_a_vars.CompareCCR1andReturnTBRcb(timestamp);
   } else {
      if (taiv_local==0x0004) {
          timer_a_vars.compareCCR2Cb();
      } else {
          if (taiv_local==0x000a){
              if (timer_a_vars.overflowCb!=_NULL) {
                  timer_a_vars.overflowCb();
              } 
          } else {
          }
      }
   }
#ifdef PIN_DEBUG
   P3OUT &= ~0x10;
#endif
   __bic_SR_register_on_exit(CPUOFF);
}