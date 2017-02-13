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
void averageSubticks();

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
    P3DIR |=  0x10;      // [P3.4]
    
    timer_a_init();
    timer_b_init();
    spi_init();

    // get eui address
    eui64_get(&address[0]);
    app_vars.myId = address[7];
   
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
    if (app_vars.myId != DESTINATION_ID){
        cc2420_spiWriteReg(
          CC2420_MDMCTRL0_ADDR,
          &cc2420_statusByte,
          0x2af2
        );
    } else {
        // no auto ack on destination side, for experiment
        cc2420_spiWriteReg(
          CC2420_MDMCTRL0_ADDR,
          &cc2420_statusByte,
          0x22e2
        );
    }

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
        // DESTINATION no auto ack, temperal setting
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
        P3OUT   ^= 0x10;
        TACCR2   =  TAR+TIMER_A_PERIOD;
        TACCTL2  =  CCIE;
    }
}

// for calculating sub ticks
void timer_a_cb_subtickCalculate(uint16_t timestamp){
    uint16_t currentValue;
    if (app_vars.isBusyCalculating==0){
        app_vars.timerStartAt = timestamp;
        // for calculating subticks
        currentValue = TACCR1;
        TACCR1       =  currentValue+TIMER_A_SUBTICK;
        TACCTL1      =  CCIE;
        app_vars.isBusyCalculating = 1;
    } else {
        uint16_t offset = (timestamp-app_vars.timerStartAt)>>8;// divide by 256: TIMER_A_SUBTICK
        app_vars.subticks[app_vars.subticks_index] = offset;
        app_vars.subticks_index = (app_vars.subticks_index+1)%16;
        averageSubticks();
        app_vars.isBusyCalculating = 0;
    }
}

void timer_b_cb_startFrame(uint16_t timestamp){
}
void timer_b_cb_endFrame(uint16_t timestamp){
    uint8_t             packet_len;
    cc2420_status_t     statusByte;
    uint8_t             dsn;
    
    if (app_vars.myId == SOURCE_ID){
         // cancel armed timer
         cc2420_spiStrobe(CC2420_SRFOFF, &app_vars.cc2420_status);
         return;
    }
    
    if (app_vars.myId==DESTINATION_ID){
        cc2420_spiReadRxFifo(&statusByte, &app_vars.packetRx[0], &packet_len, 9);
        if (packet_len>4&&packet_len<=9){
            app_vars.rxpk_crc = (app_vars.packetRx[packet_len-1]&0x80)>>7;
            if (packet_len==9 && app_vars.rxpk_crc){
                if (app_vars.packetRx[5] == 0xBB && app_vars.packetRx[6] == 0xBB){
                    P3OUT   ^= 0x10;
                }
            }
        }
        return;
    }
    
    if (app_vars.needScedule==1){
        // endOfAck needs 56us to finish, schedule a little more than this. (3 indicates 91.5us)
        TBCCR2   =  timestamp+4*app_vars.aveSubticks;
        TBCCTL2  =  CCIE;
        // turn radio off
        cc2420_spiStrobe(CC2420_SRFOFF, &app_vars.cc2420_status);
        cc2420_spiStrobe(CC2420_STXCAL, &app_vars.cc2420_status);
        app_vars.needScedule = 0;
        P3OUT ^=  0x20;
        return;
    }
    
    // just read length
    cc2420_spiReadRxFifo_length(&statusByte, &app_vars.packetRx[0], &packet_len, 9);
    cc2420_spiReadRam(0x0081,&app_vars.cc2420_status,&app_vars.packetRx[1],3);
    if (app_vars.packetRx[1]==FRAME_CONTROL_BYTE0 && app_vars.packetRx[2]==FRAME_CONTROL_BYTE1){
        dsn = app_vars.packetRx[3];
        if (dsn > app_vars.currentDsn || app_vars.currentDsn-dsn > 0xF7){
            app_vars.currentDsn = dsn;
            if (packet_len!=9){
                // write new dsn in txbuffer, len at addr 0x000, 2 byte fcf is at 0x001 and dsn is located at addr 0x003
                cc2420_spiWriteRam(0x0003,&app_vars.cc2420_status,&app_vars.currentDsn,1);
                app_vars.needScedule=1;
            }
        }
    }
}
void timer_b_cb_compareCb(void){
}
// ==== helper ====
void averageSubticks(){
    uint8_t i,j=0;
    uint16_t sum=0;
    for (i=0;i<16;i++){
        // at 3V, the typical Frequency ranges from 4.4MHz to 5.4MHz,
        // so the theoretical value of subticks ranges from (134~165)
        if (app_vars.subticks[i]>=134 && app_vars.subticks[i]<=165){
            sum += (uint16_t)app_vars.subticks[i];
            j++;
        }
    }
    if ((sum/j)>0){
        app_vars.aveSubticks=sum/j;
    } else {
        //!! something wrong, using typical value of frequency (4.9MHz)
        app_vars.aveSubticks = 149;
    }
}