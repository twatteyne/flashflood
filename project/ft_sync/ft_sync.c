#include "stdint.h"
#include "string.h"
#include "board.h"
#include "debugpins.h"
#include "leds.h"
#include "bsp_timer.h"
#include "radiotimer.h"
#include "eui64.h"
#include "radio.h"
#include "cc2420.h"
#include "msp430f1611.h"
#include "ft_sync.h"

//=========================== defines =========================================

#define CHANNEL               26

//==== frame content
#define FRAME_CONTROL_BYTE0   0x61 // 0b0110 0001  |bit6: panId compressed|bit5: AR set|bit4: no frame pending|bit3: sec disable|bit0-2: frame type,data|
#define FRAME_CONTROL_BYTE1   0x18 // 0b0001 1000  |bit14-15: src addr is elided|bit12-13:frame version, may not useful|bit10-11:16-bit dest addr|
// 2B fcf + 1B dsn + 2B dest panId + 2B dest address + 2B crc
#define FRAME_LENGTH 2 + 1 + 2 + 2 + 2 

//==== mote role
#define SOURCE_ID                          0x16
#define FIRST_HOP_1                        0xdd
#define FIRST_HOP_2                        0x0f
#define SECOND_HOP_1                       0x5e
#define SECOND_HOP_2                       0x05
#define DESTINATION_ID                     0x57

//==== timing @ ~4.9MHz
#define TxOffset                           5000 // 5000 @5MHz ~ 1000us
#define TxGuardTime                         750 // 750 @5MHz ~ 150us
#define TxAckOffset                        8350 // 670us (3350 @5MHz) from Tx to Ack start of frame, so 3350+TxOffset

#define TxAutoAckTimeout                   1750 // measure 12tickes@32kHz ~ 349us from endOfframe(data) to transmit ack
#define RxDataTimeout                      2500 // 500us, packet sent require 320us  ~ 11ticks@32kHz
#define RxAckTimeout                       1500 // 300us, ack sent require 160us

#define RxCalibrationDelay                 1555 // 12 symbals period + one spi transcation, 311us

//=== subTick calculation
#define BSP_TIMER_PERIOD                     10  // 10@32kHz ~ 305us
#define NEXT_PACKET_SCHEDULE                 10 // ticks@32kHz, it will be convert to ticks@5mHz
#define SAMPLE_SET_SIZE                       5

//==== sync
#define RESYNCHRONIZATIONGUARD              750 

//=========================== variables =======================================

static  uint8_t cc2420_shortadr_cycle_1[2] = {0xaa,0xaa};
static  uint8_t cc2420_shortadr_cycle_2[2] = {0xbb,0xbb};
static  uint8_t cc2420_panid_cycle_1[2]    = {0xaa,0xaa};
static  uint8_t cc2420_panid_cycle_2[2]    = {0xbb,0xbb};
static  uint8_t cc2420_ieeeadr_cycle_1[8]  = {0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};
static  uint8_t cc2420_ieeeadr_cycle_2[8]  = {0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb};
typedef enum {
   S_SLEEP                   = 0x00,   // ready for next slot
   // synchronizing
   S_SYNCLISTEN              = 0x01,   // listened for packet to synchronize to network
   S_SYNCRX                  = 0x02,   // receiving packet to synchronize to network
   S_SYNCPROC                = 0x03,   // processing packet just received
   // TX
   S_TXDATAOFFSET            = 0x04,   // waiting to prepare for Tx data
   S_TXDATAPREPARE           = 0x05,   // preparing for Tx data
   S_TXDATAREADY             = 0x06,   // ready to Tx data, waiting for 'go'
   S_TXDATADELAY             = 0x07,   // 'go' signal given, waiting for SFD Tx data
   S_TXDATA                  = 0x08,   // Tx data SFD received, sending bytes
   S_RXACKOFFSET             = 0x09,   // Tx data done, waiting to prepare for Rx ACK
   S_RXACKPREPARE            = 0x0a,   // preparing for Rx ACK
   S_RXACKREADY              = 0x0b,   // ready to Rx ACK, waiting for 'go'
   S_RXACKLISTEN             = 0x0c,   // idle listening for ACK
   S_RXACK                   = 0x0d,   // Rx ACK SFD received, receiving bytes
   S_TXPROC                  = 0x0e,   // processing sent data
   // RX
   S_RXDATAOFFSET            = 0x0f,   // waiting to prepare for Rx data
   S_RXDATAPREPARE           = 0x10,   // preparing for Rx data
   S_RXDATAREADY             = 0x11,   // ready to Rx data, waiting for 'go'
   S_RXDATALISTEN            = 0x12,   // idle listening for data
   S_RXDATA                  = 0x13,   // data SFD received, receiving more bytes
   S_TXACKOFFSET             = 0x14,   // waiting to prepare for Tx ACK
   S_TXACKPREPARE            = 0x15,   // preparing for Tx ACK
   S_TXACKREADY              = 0x16,   // Tx ACK ready, waiting for 'go'
   S_TXACKDELAY              = 0x17,   // 'go' signal given, waiting for SFD Tx ACK
   S_TXACK                   = 0x18,   // Tx ACK SFD received, sending bytes
   S_RXPROC                  = 0x19,   // processing received data
} app_state_t;

typedef struct {
    uint16_t asn0and1;
    uint16_t asn2and3;
}asn_t;

typedef struct {
    bool                changeDetected;
    bool                isSrc;
    bool                isDest;
    app_state_t         app_state;
    bool                isSync;
    int8_t              rxpk_rssi;
    uint8_t             rxpk_lqi;
    bool                rxpk_crc;
    bool                f_SFDreceived;
    cc2420_status_t     cc2420_status;
    uint8_t             packet[FRAME_LENGTH];
    uint16_t            timeReceivedData;
    uint16_t            timeReceivedAck;
    uint8_t             lastDsn;
    asn_t               asn;
    
    uint16_t            lastCouter;
    uint16_t            ticksAt5m2oneTickAt32k[SAMPLE_SET_SIZE];
    uint16_t            internalValue;
    uint8_t             tickCounter_index;
    bool                packetScheduled;
    
    uint8_t             cycleId;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

// bso level
void bsp_timer_cb_compare(void);
void radiotimer_cb_overflow(void);
void radiotimer_cb_compare(void);
void radiotimer_cb_startFrame(PORT_RADIOTIMER_WIDTH timestamp);
void radiotimer_cb_endFrame(PORT_RADIOTIMER_WIDTH timestamp);

// interrupt handlers
kick_scheduler_t   radiotimer_isr(void);

// synchronization
void increaseAsn(void);
void synchronizePacket(PORT_RADIOTIMER_WIDTH timeReceived, PORT_RADIOTIMER_WIDTH reference);
void endSlot();

// helper
uint16_t averageArray(uint16_t* array,uint8_t length);

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
    uint8_t address[8];
    memset(&app_vars,0,sizeof(app_vars_t));
    memset(&address[0],0,8);
   
    // initialize board
    board_init();
    // get eui address
    eui64_get(&address[0]);
   
    // switch radio LED on
    leds_radio_on();
   
    // prepare bsp_timer
    bsp_timer_set_callback(bsp_timer_cb_compare);
   
    // prepare radiotimer
    radio_setOverflowCb(radiotimer_cb_overflow);
    radio_setCompareCb(radiotimer_cb_compare);
    radio_setStartFrameCb(radiotimer_cb_startFrame);
    radio_setEndFrameCb(radiotimer_cb_endFrame);
   
    // prepare radio
    radio_rfOn();
    radio_setFrequency(CHANNEL);
    
    // ==== write short addr, panid and ieee addr
    if (
        address[7]==SOURCE_ID   ||
        address[7]==FIRST_HOP_1 ||
        address[7]==FIRST_HOP_2
    ) {
        // write short address
        cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR,
                           &app_vars.cc2420_status,
                           &cc2420_shortadr_cycle_1[0],
                           2);
        // write panId
        cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,
                           &app_vars.cc2420_status,
                           &cc2420_panid_cycle_1[0],
                           2);
        // write 64-bit ieee address
        cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,
                           &app_vars.cc2420_status,
                           &cc2420_ieeeadr_cycle_1[0],
                           8);
        // fill packet
        app_vars.packet[0] = FRAME_CONTROL_BYTE0; // fcf byte0
        app_vars.packet[1] = FRAME_CONTROL_BYTE1; // fcf byte1
//        app_vars.packet[2] = app_vars.lastDsn;    // fill it at beginning of each slot
        app_vars.packet[3] = 0xaa;                // panId, LITTLE_ENDIAN
        app_vars.packet[4] = 0xaa;                
        app_vars.packet[5] = 0xaa;                // destAddr, LITTLE_ENDIAN
        app_vars.packet[6] = 0xaa;                
        app_vars.packet[7] = 0x00;                // reserved for crc
        app_vars.packet[8] = 0x00;
        app_vars.cycleId   = 0xaa;
    }
    
    if (
        address[7]==DESTINATION_ID   ||
        address[7]==SECOND_HOP_1     ||
        address[7]==SECOND_HOP_2
    ) {
        // write short address
        cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR,
                           &app_vars.cc2420_status,
                           &cc2420_shortadr_cycle_2[0],
                           2);
        // write panId
        cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,
                           &app_vars.cc2420_status,
                           &cc2420_panid_cycle_2[0],
                           2);
        // write 64-bit ieee address
        cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,
                           &app_vars.cc2420_status,
                           &cc2420_ieeeadr_cycle_2[0],
                           8);
                // fill packet
        app_vars.packet[0] = FRAME_CONTROL_BYTE0; // fcf byte0
        app_vars.packet[1] = FRAME_CONTROL_BYTE1; // fcf byte1
//        app_vars.packet[2] = app_vars.lastDsn;    // fill it at beginning of each slot
        app_vars.packet[3] = 0xbb;                // panId, LITTLE_ENDIAN
        app_vars.packet[4] = 0xbb;                
        app_vars.packet[5] = 0xbb;                // destAddr, LITTLE_ENDIAN
        app_vars.packet[6] = 0xbb;                
        app_vars.packet[7] = 0x00;                // reserved for crc
        app_vars.packet[8] = 0x00;
        app_vars.cycleId   = 0xbb;
    }
   
    if (address[7]==SOURCE_ID){
        app_vars.isSrc   = 1;
        app_vars.isSync  = 1;
        app_vars.changeDetected = 1;
        leds_sync_on();
        app_vars.lastDsn = 1; // start dsn from 1 if I am source mote
    } else {
        if (address[7]==DESTINATION_ID){
            app_vars.isDest   = 1;
        }
        app_vars.lastDsn = 0; // start dsn from 0 if I am not source mote
    }
    
    // start periodic radiotimer overflow
    radiotimer_start(PORT_TsSlotDuration);
   
    while (1) {
        board_sleep();
    }
}

//=========================== callbacks =======================================

void bsp_timer_cb_compare(void) {
    uint16_t currentCounter;
    // record radtimer counter
    currentCounter              = radiotimer_getValue();
    app_vars.ticksAt5m2oneTickAt32k[app_vars.tickCounter_index] = (currentCounter-app_vars.lastCouter)/BSP_TIMER_PERIOD;
    app_vars.tickCounter_index  = (app_vars.tickCounter_index+1)%SAMPLE_SET_SIZE;
    
    app_vars.internalValue  = averageArray(&app_vars.ticksAt5m2oneTickAt32k[0],SAMPLE_SET_SIZE);
    app_vars.internalValue *= NEXT_PACKET_SCHEDULE;
    app_vars.internalValue -= PORT_delayTx;
}

void radiotimer_cb_overflow(void) {
    // new slot
   
    if (app_vars.isSync){
        debugpins_slot_toggle();
        radiotimer_setPeriod(PORT_TsSlotDuration);
        increaseAsn();
        debugpins_fsm_toggle();
        if (app_vars.isSrc){
            if (app_vars.changeDetected){
                app_vars.app_state = S_TXDATAPREPARE;
                // schedule for sending packet
                radiotimer_schedule(TxOffset-PORT_delayTx);
                // preparing Tx
                app_vars.packet[2] = app_vars.lastDsn;
                app_vars.lastDsn++;
            } else {
                if (app_vars.packetScheduled==1){
                    app_vars.app_state = S_TXDATAPREPARE;
                } else {
                    endSlot();
                    return;
                }
            }
        } else {
            if (app_vars.packetScheduled==0){
                app_vars.app_state = S_RXDATAPREPARE;
                radiotimer_schedule(TxOffset-TxGuardTime-RxCalibrationDelay);
                radio_rxEnable();
                app_vars.app_state = S_RXDATAREADY;
                return;
            } else {
                app_vars.app_state = S_TXDATAPREPARE;
            }
        }
                // flush rxfifo
        cc2420_spiStrobe(CC2420_SFLUSHRX, &app_vars.cc2420_status);
        // stop listening
        radio_rfOff();
        // start transmitting packet
        radio_loadPacket(app_vars.packet,FRAME_LENGTH);
        radio_txEnable();
        app_vars.app_state = S_TXDATAREADY;
    } else {
        if (app_vars.app_state == S_RXDATA ||app_vars.app_state == S_TXACK){
            // in the middle of receiving something right now
            return;
        }
        // keep listening for packet
        app_vars.app_state = S_RXDATALISTEN;
        radio_rxNow();
    }
}

void radiotimer_cb_compare(void) {
    // toggle pin
    debugpins_fsm_toggle();
    switch (app_vars.app_state){
    case S_TXDATAREADY:
        app_vars.app_state = S_TXDATADELAY;
        radio_txNow();
        break;
    case S_RXDATAREADY:
        app_vars.app_state = S_RXDATALISTEN;
        radiotimer_schedule(TxOffset+TxGuardTime);
        radio_rxNow();
        break;
    case S_RXDATALISTEN:
        // nothing heard, try to listen auto Ack
        app_vars.app_state = S_RXACKPREPARE;
        radiotimer_cancel();
        radio_rfOff();
        radiotimer_schedule(TxAckOffset-TxGuardTime-RxCalibrationDelay);
        app_vars.app_state = S_RXACKREADY;
        break;
    case S_RXACKREADY:
        app_vars.app_state = S_RXACKLISTEN;
        radiotimer_schedule(TxAckOffset+TxGuardTime);
        radio_rxNow();
        break;
    case S_RXACKLISTEN:
        // nothing heard, end slot
        endSlot();
        break;
    case S_RXDATA:
        // rx data take too long
        endSlot();
        break;
    default:
        // indicate error and end the slot
        leds_error_toggle();
        endSlot();
    }
}

void radiotimer_cb_startFrame(PORT_RADIOTIMER_WIDTH timestamp){
    switch (app_vars.app_state){
    case S_TXDATADELAY:
        app_vars.app_state  = S_TXDATA;
        break;
    case S_RXDATALISTEN:
        app_vars.timeReceivedData = timestamp;
        app_vars.app_state    = S_RXDATA;
        radiotimer_schedule(timestamp+RxDataTimeout);
        break;
    case S_RXACKLISTEN:
        app_vars.timeReceivedAck = timestamp;
        app_vars.app_state    = S_RXACK;
        radiotimer_schedule(timestamp+RxAckTimeout);
        break;
    case S_TXACKDELAY:
        app_vars.app_state  = S_TXACK;
        break;
    default:
        // indicate error and endslot
        leds_error_toggle();
        endSlot();
    }
}

void radiotimer_cb_endFrame(PORT_RADIOTIMER_WIDTH timestamp){
    uint8_t packet_len;
    switch (app_vars.app_state){
    case S_RXACK:
    case S_TXACK:
        // this may be end of txack or rxack
      
        // schedule the packet at next slot if ack has new dsn insdie
        // if not needed, cancel it later
        radiotimer_schedule(app_vars.internalValue-(TBCCR0-timestamp));
        // check received packet
        memset(&app_vars.packet,0,sizeof(app_vars.packet));
        // get packet from radio
        radio_getReceivedFrame(
            app_vars.packet,
            &packet_len,
            sizeof(app_vars.packet),
            &app_vars.rxpk_rssi,
            &app_vars.rxpk_lqi,
            &app_vars.rxpk_crc
        );
        if (packet_len==5){
            // this is Ack
            radio_rfOff();
        } else {
            // this is data
            if (app_vars.cycleId != app_vars.packet[3]){
                app_vars.app_state = S_RXDATALISTEN;
                return;
            }
        }
        if (app_vars.isSrc==0){
            if (packet_len==5){
                app_vars.packetScheduled = 1;
            }
            if (app_vars.isSync==0){
                // update last dsn 
                app_vars.lastDsn = app_vars.packet[2];  
                if (packet_len==5){
                    synchronizePacket(app_vars.timeReceivedAck,TxAckOffset);
                } else {
                    synchronizePacket(app_vars.timeReceivedData,TxOffset);
                    // no need to schedule data
                    endSlot();
                }
            } else {
                // check whether this is first time heard the packet depending on dsn
                if (
                    (
                      (app_vars.packet[2]>app_vars.lastDsn)
                    ) ||
                    (
                      app_vars.packet[2] < app_vars.lastDsn &&
                      app_vars.lastDsn - app_vars.packet[2] > 0xf7 // in case I lost few packets previous
                    )
                ){
                    // update last dsn
                    app_vars.lastDsn = app_vars.packet[2];
                    if (packet_len==5){
                        // synchronization
                        synchronizePacket(app_vars.timeReceivedAck,TxAckOffset);
                    } else {
                        // synchronization
                        synchronizePacket(app_vars.timeReceivedData,TxOffset);
                        // no need to schedule data packet
                        endSlot();
                    }
                } else {
                    // this is packet I heared or already relayed
                    app_vars.packetScheduled = 0;
                    endSlot();
                }
            }
        } else {
            // src never relay packet
            endSlot();
        }
        break;
    case S_TXDATA:
        // after transmission, radio will turn to Rx itself
        app_vars.app_state  = S_RXACKLISTEN;
        radiotimer_schedule(timestamp+TxAutoAckTimeout);
        break;
    case S_RXDATA:
        radiotimer_cancel();
        // expected an ACK will be sent automatically, however, if this is not
        // valid data for me, I will disregard packet and searching for another,
        // which probably will be an Ack later. Use TxACKDELAY state here, 
        // after end of frame, i will check depending on received frame. 
        app_vars.app_state = S_TXACKDELAY;
        break;
    default:
        // indicate error and endof slot
        leds_error_toggle();
        endSlot();
    }
}

//==== synchronization

void increaseAsn(void){
    if (app_vars.asn.asn0and1==0xffff){
        app_vars.asn.asn2and3++;
        app_vars.asn.asn0and1 = 0;
    } else {
        app_vars.asn.asn0and1++;
    }
}

void synchronizePacket(PORT_RADIOTIMER_WIDTH timeReceived, PORT_RADIOTIMER_WIDTH reference) {
   PORT_SIGNED_INT_WIDTH timeCorrection;
   PORT_RADIOTIMER_WIDTH newPeriod;
   PORT_RADIOTIMER_WIDTH currentPeriod;
   PORT_RADIOTIMER_WIDTH currentValue;
   
   // record the current timer value and period
   currentValue                   =  radio_getTimerValue();
   currentPeriod                  =  radio_getTimerPeriod();
   
   // calculate new period
   timeCorrection                 =  (PORT_SIGNED_INT_WIDTH)((PORT_SIGNED_INT_WIDTH)timeReceived - (PORT_SIGNED_INT_WIDTH)reference);


   // The interrupt beginning a new slot can either occur after the packet has been
   // or while it is being received, possibly because the mote is not yet synchronized.
   // In the former case we simply take the usual slotLength and correct it.
   // In the latter case the timer did already roll over and
   // currentValue < timeReceived. slotLength did then already pass which is why
   // we need the new slot to end after the remaining time which is timeCorrection
   // and in this constellation is guaranteed to be positive.
   if (currentValue < timeReceived) {
       newPeriod = (PORT_RADIOTIMER_WIDTH)timeCorrection;
   } else {
       newPeriod =  (PORT_RADIOTIMER_WIDTH)((PORT_SIGNED_INT_WIDTH)currentPeriod + timeCorrection);
   }
   
   // detect whether I'm too close to the edge of the slot, in that case,
   // skip a slot and increase the temporary slot length to be 2 slots long
   if ((PORT_SIGNED_INT_WIDTH)newPeriod - (PORT_SIGNED_INT_WIDTH)currentValue < (PORT_SIGNED_INT_WIDTH)RESYNCHRONIZATIONGUARD) {
      newPeriod                  +=  PORT_TsSlotDuration;
      increaseAsn();
   }
   
   // resynchronize by applying the new period
   radio_setTimerPeriod(newPeriod);
   
   // update as synced
    app_vars.isSync = 1;
    leds_sync_on();
}
void endSlot(){
    memset(&app_vars.packet,0,sizeof(app_vars.packet));
    // fill packet
    app_vars.packet[0] = FRAME_CONTROL_BYTE0; // fcf byte0
    app_vars.packet[1] = FRAME_CONTROL_BYTE1; // fcf byte1
//        app_vars.packet[2] = app_vars.lastDsn;    // fill it at beginning of each slot
    app_vars.packet[3] = app_vars.cycleId;
    app_vars.packet[4] = app_vars.cycleId;
    app_vars.packet[5] = app_vars.cycleId;
    app_vars.packet[6] = app_vars.cycleId;
    app_vars.packet[7] = 0x00;                
    app_vars.packet[8] = 0x00;
    
    radio_rfOff();
    radiotimer_cancel();
    
    // schedule bsp timer for calibrating SMCLK clock
    bsp_timer_reset();
    app_vars.lastCouter = radiotimer_getValue();
    bsp_timer_scheduleIn(BSP_TIMER_PERIOD);
}

// ========================== helper ==========================================

// make sure the item value is less than (65535/length)
uint16_t averageArray(uint16_t* array,uint8_t length){
    uint8_t i,j=0;
    uint16_t average, sum=0;
    for (i=0;i<length;i++){
        // don't take zero into consideration
        if (array!=0){
            sum += array[i];
            j++;
        }
    }
    average = (sum/j);
    return average;
}

void ft_sync_changeDetected(){
    app_vars.changeDetected = 1;
}