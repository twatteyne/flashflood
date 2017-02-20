#ifndef __TIMER_A_H
#define __TIMER_A_H

#include "stdint.h"

//=========================== define ==========================================

//=========================== typedef =========================================

typedef void (*timer_a_cbt)(void);
typedef void (*timer_a_capture_cbt)(uint16_t timestamp);

//=========================== variables =======================================

//=========================== prototypes ======================================

void               timer_a_setOverflowCb(timer_a_cbt cb);
void               timer_a_setCompareCCR2Cb(timer_a_cbt cb);
void               timer_a_setCompareCCR1andReturnTBRcb(timer_a_capture_cbt cb);

/**
\}
\}
*/

#endif
