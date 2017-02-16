#include "stdint.h"
#include "opendefs.h"
//=========================== defines =========================================

#define LIGHT_SAMPLE_PERIOD                    100 // 3ms

typedef struct {
    uint8_t             rxpk_crc;
    cc2420_status_t     cc2420_status;
    uint8_t             packetTx[FRAME_LENGTH];
    
    uint8_t             myId;
    uint8_t             currentDsn;
    uint8_t             myRank;
    
    uint16_t            subticks;
    uint16_t            lastTimestamp;
    
    uint8_t             light_state;
    uint16_t            light_reading;
} app_vars_t;

//=========================== prototype =======================================