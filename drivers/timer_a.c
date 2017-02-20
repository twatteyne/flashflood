#include "msp430f1611.h"
#include "timer_a.h"
#include "string.h"

//=========================== defines =========================================

//=========================== variables =======================================

typedef struct {
   timer_a_cbt          overflowCb;
   timer_a_cbt          compareCCR2Cb;
   timer_a_capture_cbt  compareCCR1andReturnTBRcb;
   uint16_t             last_compare_value;
} timer_a_vars_t;

timer_a_vars_t timer_a_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================

void timer_a_setOverflowCb(timer_a_cbt cb) {
    timer_a_vars.overflowCb           = cb;
}

void timer_a_setCompareCCR2Cb(timer_a_cbt cb) {
    timer_a_vars.compareCCR2Cb         = cb;
}

void timer_a_setCompareCCR1andReturnTBRcb(timer_a_capture_cbt cb) {
    timer_a_vars.compareCCR1andReturnTBRcb    = cb;
}

//=========================== interrup handlers ===============================

