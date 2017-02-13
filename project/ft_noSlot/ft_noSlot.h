#include "stdint.h"

//=========================== defines =========================================

#define CHANNEL               26

//==== frame content
#define FRAME_CONTROL_BYTE0   0x61 // 0b0110 0001  |bit6: panId compressed|bit5: AR set|bit4: no frame pending|bit3: sec disable|bit0-2: frame type,data|
#define FRAME_CONTROL_BYTE1   0x18 // 0b0001 1000  |bit14-15: src addr is elided|bit12-13:frame version, may not useful|bit10-11:16-bit dest addr|
// 2B fcf + 1B dsn + 2B dest panId + 2B dest address + 2B crc
#define FRAME_LENGTH 2 + 1 + 2 + 2 + 2 
#define ACK_LENGTH   5

//==== mote role
#define SOURCE_ID                          0x0f
#define FIRST_HOP_1                        0x05
#define FIRST_HOP_2                        0x5e
#define SECOND_HOP_1                       0x16
#define SECOND_HOP_2                       0x57
#define DESTINATION_ID                     0xdd

// linear two hop: 0f->dd->16
//#define SOURCE_ID                          0x0f
//#define FIRST_HOP_1                        0xdd
//#define FIRST_HOP_2                        0x5e
//#define SECOND_HOP_1                       0x16
//#define SECOND_HOP_2                       0x11
//#define DESTINATION_ID                     0x11

// one hop
//#define SOURCE_ID                          0x57
//#define FIRST_HOP_1                        0xdd
//#define FIRST_HOP_2                        0x16
//#define SECOND_HOP_1                       0x11
//#define SECOND_HOP_2                       0x11
//#define DESTINATION_ID                     0x11

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
    
    uint8_t             subticks[16];
    uint8_t             subticks_index;
    uint8_t             aveSubticks;
    uint16_t            timerStartAt;
    uint8_t             isBusyCalculating;
    
    uint8_t             cycleId;
    uint8_t             needScedule;
} app_vars_t;

//=========================== prototype =======================================

void timer_a_cb_subtickCalculate(uint16_t timestampe);
void timer_a_cb_overflow(void);
void timer_a_cb_compare(void);
void timer_b_cb_startFrame(uint16_t timestamp);
void timer_b_cb_endFrame(uint16_t timestamp);
void timer_b_cb_compareCb(void);