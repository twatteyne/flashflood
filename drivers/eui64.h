/**
\brief Driver which retrieves the board's EUI-64 identifier.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, March 2012.
*/

#ifndef __EUI64_H
#define __EUI64_H

#include <stdint.h>
 
//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void eui64_get(uint8_t* addressToWrite);

#endif