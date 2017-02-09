#ifndef __RADIOTIMER_H
#define __RADIOTIMER_H

/**
\addtogroup BSP
\{
\addtogroup radiotimer
\{

\brief Cross-platform declaration "radiotimer" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#include "stdint.h"
#include "board.h"

//=========================== define ==========================================

//=========================== typedef =========================================

typedef void (*radiotimer_compare_cbt)(void);
typedef void (*radiotimer_capture_cbt)(PORT_TIMER_WIDTH timestamp);

//=========================== variables =======================================

//=========================== prototypes ======================================

// admin
void     radiotimer_init(void);
void     radiotimer_setCompareCb(radiotimer_compare_cbt cb);
void     radiotimer_setStartFrameCb(radiotimer_capture_cbt cb);
void     radiotimer_setEndFrameCb(radiotimer_capture_cbt cb);
void     radiotimer_start(PORT_RADIOTIMER_WIDTH period);

void     radiotimer_scheduleIn(PORT_RADIOTIMER_WIDTH offset);
void     radiotimer_cancel();
void     radiotimer_reset();

// direct access
PORT_RADIOTIMER_WIDTH radiotimer_getValue(void);

// interrupt handlers
kick_scheduler_t   radiotimer_isr(void);

/**
\}
\}
*/

#endif
