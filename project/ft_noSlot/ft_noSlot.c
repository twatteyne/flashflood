#include "timer_a.h"
#include "timer_b.h"
#include "eui64.h"
#include "spi.h"
#include "radio.h"
#include "cc2420.h"
#include "msp430f1611.h"
#include "ft_noSlot.h"
#include "string.h"

//=========================== variables =======================================

static  uint8_t cc2420_shortadr_cycle_1[2] = {0xaa,0xaa};
static  uint8_t cc2420_shortadr_cycle_2[2] = {0xbb,0xbb};
static  uint8_t cc2420_shortadr_cycle_3[2] = {0xcc,0xcc};
static  uint8_t cc2420_panid_cycle_1[2]    = {0xaa,0xaa};
static  uint8_t cc2420_panid_cycle_2[2]    = {0xbb,0xbb};
static  uint8_t cc2420_panid_cycle_3[2]    = {0xcc,0xcc};
static  uint8_t cc2420_ieeeadr_cycle_1[8]  = {0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};
static  uint8_t cc2420_ieeeadr_cycle_2[8]  = {0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb};
static  uint8_t cc2420_ieeeadr_cycle_3[8]  = {0xcc,0xcc,0xcc,0xcc,0xcc,0xcc,0xcc,0xcc};

app_vars_t app_vars;

//=========================== prototypes ======================================
// helper
uint16_t averageArray(uint16_t* array,uint8_t length);

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int main(void) {
    uint8_t i,address[8];
    volatile uint16_t     delay;
    cc2420_status_t cc2420_statusByte;
    
    memset(&app_vars,0,sizeof(app_vars_t));
    memset(&address[0],0,8);
    
        // disable watchdog timer
    WDTCTL     =  WDTPW + WDTHOLD;
   
    // setup clock speed
    DCOCTL    |=  DCO0 | DCO1 | DCO2;             // MCLK at ~8MHz
    BCSCTL1   |=  RSEL0 | RSEL1 | RSEL2;          // MCLK at ~8MHz
                                                 // by default, ACLK from 32kHz XTAL which is running
    // initialize pins
    P4DIR     |=  0x20;                           // [P4.5] radio VREG:  output
    P4DIR     |=  0x40;                           // [P4.6] radio reset: output
    
    P6DIR |=  0x40;      // [P6.6]
    P6DIR |=  0x80;      // [P6.7]
    P2DIR |=  0x08;      // [P2.3]
    P2DIR |=  0x40;      // [P2.6]
    P3DIR |=  0x20;      // [P3.5]
    
    timer_a_init();
    timer_b_init();
    spi_init();
   
    // set radio VREG pin high
    P4OUT |=  0x20;
    for (delay=0xffff;delay>0;delay--);           // max. VREG start-up time is 0.6ms

    // set radio RESET pin low
    P4OUT &= ~0x40; 
    for (delay=0xffff;delay>0;delay--);

    // set radio RESET pin high
    P4OUT |=  0x40;
    for (delay=0xffff;delay>0;delay--);

    // 3 leading zero's (IEEE802.15.4 compliant)
    // turn on auto ack
    // turn on auto crc 
    // turn On address recognition 
    // accept all frame types
    cc2420_spiWriteReg(
      CC2420_MDMCTRL0_ADDR,
      &cc2420_statusByte,
      0x2af2
    );

    // speed up time to TX
    // max. TX power (~0dBm), faster STXON->SFD timing (128us)
    cc2420_spiWriteReg(
      CC2420_TXCTRL_ADDR,
      &cc2420_statusByte,
      0x80ff
    );

    // apply correction recommended in datasheet
    cc2420_spiWriteReg(
      CC2420_RXCTRL1_ADDR,
      &cc2420_statusByte,
      0x2a56
    );

    // get eui address
    eui64_get(&address[0]);
    app_vars.myId = address[7];

    timer_a_setSubtickCalculateCb(timer_a_cb_subtickCalculate);
    timer_a_setOverflowCb(timer_a_cb_overflow);
    timer_a_setCompareCb(timer_a_cb_compare);
    timer_b_setStartFrameCb(timer_b_cb_startFrame);
    timer_b_setEndFrameCb(timer_b_cb_endFrame);
    
    __bis_SR_register(GIE);

    // ==== write short addr, panid and ieee addr
    // prepare radio
    radio_rfOn();
    if (
        app_vars.myId==SOURCE_ID   ||
        app_vars.myId==FIRST_HOP_1 ||
        app_vars.myId==FIRST_HOP_2
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
        app_vars.cycleId   = 0xaa;
    }

    if (
        app_vars.myId==SECOND_HOP_1     ||
        app_vars.myId==SECOND_HOP_2
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
        app_vars.cycleId   = 0xbb;
    }

    if (app_vars.myId==DESTINATION_ID){
        // write short address
        cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR,
                           &app_vars.cc2420_status,
                           &cc2420_shortadr_cycle_3[0],
                           2);
        // write panId
        cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,
                           &app_vars.cc2420_status,
                           &cc2420_panid_cycle_3[0],
                           2);
        // write 64-bit ieee address
        cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,
                           &app_vars.cc2420_status,
                           &cc2420_ieeeadr_cycle_3[0],
                           8);
        app_vars.cycleId   = 0xcc;
    }

    //fcf is always the same
    app_vars.packetTx[0] = FRAME_CONTROL_BYTE0; // fcf byte0
    app_vars.packetTx[1] = FRAME_CONTROL_BYTE1; // fcf byte1
    for (i=0;i<4;i++){
        app_vars.packetTx[i+3] = app_vars.cycleId; 
    }
    radio_setFrequency(CHANNEL);
    radio_loadPacket(app_vars.packetTx,FRAME_LENGTH);
    
    if (app_vars.myId==SOURCE_ID){
        // schedule timer for sending
        TACCR2   =  TAR+TIMER_A_PERIOD;
        TACCTL2  =  CCIE;
    }
    
    radio_rxNow();

    while (1) {
    }
}

//=========================== callbacks =======================================

void timer_a_cb_overflow(void) {
}
void timer_a_cb_compare(void) {
    if (app_vars.myId==SOURCE_ID){
        cc2420_status_t cc2420_statusByte;
        app_vars.currentDsn++;
        cc2420_spiWriteRam(0x0003,&app_vars.cc2420_status,&app_vars.currentDsn,1);
        // tx now
        cc2420_spiStrobe(CC2420_STXON, &cc2420_statusByte);
        TACCR2   =  TAR+TIMER_A_PERIOD;
        TACCTL2  =  CCIE;
    }
}

void timer_a_cb_subtickCalculate(uint16_t timestamp){
    uint16_t offset = timestamp>>8;// divide by 128: TIMER_A_SUBTICK
    timer_b_setOffset(offset*15);  // endOfAck needs 371us to finish, schedule a little more than this. 15 indicate 450us
}

void timer_b_cb_startFrame(uint16_t timestamp){
}
void timer_b_cb_endFrame(uint16_t timestamp){
    uint8_t             packet_len;
    cc2420_status_t     statusByte;
    uint8_t             dsn;
    
    if (app_vars.myId == SOURCE_ID){
         // cancel armed timer
         TBCCR2   =  0;
         TBCCTL2 &= ~CCIE;
         return;
    }
    
    // just read length
    cc2420_spiReadRxFifo(&statusByte, &app_vars.packetRx[0], &packet_len, 9);
    // whether CRC checked (bit 7)
    app_vars.rxpk_crc = (app_vars.packetRx[packet_len-1]&0x80)>>7;
    if (app_vars.rxpk_crc == 0 || packet_len!=ACK_LENGTH){
         // cancel armed timer
         TBCCR2   =  0;
         TBCCTL2 &= ~CCIE;
    } else {
        dsn = app_vars.packetRx[2];
        if (dsn <= app_vars.currentDsn){
            TBCCR2   =  0;
            TBCCTL2 &= ~CCIE;
        } else {
            app_vars.currentDsn = dsn;
            // write new dsn in txbuffer, len at addr 0x000, 2 byte fcf is at 0x001 and dsn is located at addr 0x003
            cc2420_spiWriteRam(0x0003,&app_vars.cc2420_status,&app_vars.currentDsn,1);
            // turn radio off
            cc2420_spiStrobe(CC2420_SRFOFF, &app_vars.cc2420_status);
            timer_b_setPacketTobeSent();
            P3OUT ^=  0x20;
        }
         
    }
}
void timer_b_cb_compareCb(void){
}