#include "stdint.h"
#include "opendefs.h"
//=========================== defines =========================================

#define TIMER_A_PERIOD                    32768 // 1s

typedef struct {
    uint8_t             rxpk_crc;
    cc2420_status_t     cc2420_status;
    uint8_t             packetTx[FRAME_LENGTH];
    
    uint8_t             myId;
    uint8_t             currentDsn;
    
    uint16_t            subticks;
    uint16_t            lastTimestamp;
} app_vars_t;

//=========================== prototype =======================================