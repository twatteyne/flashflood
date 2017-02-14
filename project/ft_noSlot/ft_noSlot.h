#include "stdint.h"
#include "opendefs.h"
//=========================== defines =========================================

#define TIMER_A_PERIOD                    32768 // 1s

typedef struct {
    int8_t              rxpk_rssi;
    uint8_t             rxpk_lqi;
    uint8_t             rxpk_crc;
    uint8_t             f_SFDreceived;
    cc2420_status_t     cc2420_status;
    uint8_t             packetTx[FRAME_LENGTH];
    uint8_t             packetRx[FRAME_LENGTH];
    
    uint8_t             myId;
    uint8_t             currentDsn;
    
    uint16_t            subticks;
    uint16_t            lastTimestamp;
    
    uint8_t             cycleId;
    uint8_t             needSchedule;
    uint8_t             needSkip;
    uint8_t             needFlushRxFIFO;
    uint8_t             debug;
} app_vars_t;

//=========================== prototype =======================================

void timer_a_cb_subtickCalculate(uint16_t timestampe);
void timer_a_cb_overflow(void);
void timer_a_cb_compare(void);
void timer_b_cb_startFrame(uint16_t timestamp);
void timer_b_cb_endFrame(uint16_t timestamp);
void timer_b_cb_compareCb(void);