#ifndef __TIMER_B_H
#define __TIMER_B_H

#include "stdint.h"

//=========================== define ==========================================

#define TIMER_A_SUBTICK                     256

//=========================== typedef =========================================

typedef void (*timer_b_compare_cbt)(void);
typedef void (*timer_b_capture_cbt)(uint16_t timestamp);

//=========================== variables =======================================

//=========================== prototypes ======================================

// admin
void     timer_b_init(void);
void     timer_b_setCompareCb(timer_b_compare_cbt cb);
void     timer_b_setStartFrameCb(timer_b_capture_cbt cb);
void     timer_b_setEndFrameCb(timer_b_capture_cbt cb);

void     timer_b_setOffset(uint16_t offset);

/**
\}
\}
*/

#endif
