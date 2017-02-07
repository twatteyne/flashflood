/**
\brief TelosB-specific board information bsp module.

This module file defines board-related element, but which are applicable only
to this board.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#ifndef __BOARD_INFO_H
#define __BOARD_INFO_H

#include "stdint.h"
#include "msp430f1611.h"
#include "string.h"

//=========================== define ==========================================

//===== interrupt state

#if defined(__GNUC__) && (__GNUC__==4)  && (__GNUC_MINOR__<=5) && defined(__MSP430__)
   // mspgcc <4.5.x
   #define INTERRUPT_DECLARATION()          unsigned short s;
   #define DISABLE_INTERRUPTS()             s = READ_SR&0x0008; \
                                            __disable_interrupt();
   #define ENABLE_INTERRUPTS()              __asm__("bis %0,r2" : : "ir" ((uint16_t) s));
#else
   // other
   #define INTERRUPT_DECLARATION()          __istate_t s;
   #define DISABLE_INTERRUPTS()             s = __get_interrupt_state(); \
                                            __disable_interrupt();
   #define ENABLE_INTERRUPTS()              __set_interrupt_state(s);
#endif

//===== timer

#define PORT_TIMER_WIDTH                    uint16_t
#define PORT_RADIOTIMER_WIDTH               uint16_t

#define PORT_SIGNED_INT_WIDTH               int16_t

//===== pins

// [P4.5] radio VREG
#define PORT_PIN_RADIO_VREG_HIGH()          P4OUT |=  0x20;
#define PORT_PIN_RADIO_VREG_LOW()           P4OUT &= ~0x20;
// [P4.6] radio RESET
#define PORT_PIN_RADIO_RESET_HIGH()         P4OUT |=  0x40;
#define PORT_PIN_RADIO_RESET_LOW()          P4OUT &= ~0x40;  

//===== IEEE802154E timing

// time-slot related
#define PORT_TsSlotDuration                 25000   // 5ms@5MHz

// radio speed related
#define PORT_delayTx                        1760    // 366us (measured  352us)@5MHz
#define PORT_delayRx                        0       //     0us (can not measure)


//=========================== prototypes ======================================

//=========================== public ==========================================

//=========================== private =========================================

#endif
