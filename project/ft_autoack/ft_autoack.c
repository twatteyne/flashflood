#include "msp430f1611.h"
#include "spi.h"
#include "radio.h"
#include "radiotimer.h"
#include "cc2420.h"
#include "debugpins.h"
#include "leds.h"
//=========================== define ==========================================

#define CHANNEL               11
#define SENDING_PERIOD        32768 // 32768@32kHz = 1 second

#define FRAME_CONTROL_BYTE0   0x61 // 0b0110 0001  |bit6: panId compressed|bit5: AR set|bit4: no frame pending|bit3: sec disable|bit0-2: frame type,data|
#define FRAME_CONTROL_BYTE1   0x18 // 0b0001 1000  |bit14-15: src addr is elided|bit12-13:frame version, may not useful|bit10-11:16-bit dest addr|
// 2B fcf + 1B dsn + 2B dest panId + 2B dest address + 2B crc
#define FRAME_LENGTH 2 + 1 + 2 + 2 + 2 

//=========================== variable ========================================

static  uint8_t cc2420_shortadr[2] = {0xaa,0xaa};
static  uint8_t cc2420_panid[2]    = {0xff,0xff};
static  uint8_t cc2420_ieeeadr[8]  = {0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};

typedef struct {
    bool                 okToSend;
    bool                 sendDone;
    uint8_t              packet[FRAME_LENGTH];
    int8_t               rxpk_rssi;
    uint8_t              rxpk_lqi;
    bool                 rxpk_crc;
    uint8_t              dsn;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_radioTimerOverflows(void);
void     cb_radioTimerCompare(void);
void     cb_startFrame(PORT_RADIOTIMER_WIDTH timestamp);
void     cb_endFrame(PORT_RADIOTIMER_WIDTH timestamp);

void     fr_autoack_loadPacket();

//=========================== main ============================================


int main(void) {
    cc2420_status_t cc2420_status;
    
    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));
  
    // disable watchdog timer
    WDTCTL     =  WDTPW + WDTHOLD;
   
    // setup clock speed
    DCOCTL    |=  DCO0 | DCO1 | DCO2;             // MCLK at ~8MHz
    BCSCTL1   |=  RSEL0 | RSEL1 | RSEL2;          // MCLK at ~8MHz
                                                 // by default, ACLK from 32kHz XTAL which is running
   
    // initialize pins
    P4DIR     |=  0x20;                           // [P4.5] radio VREG:  output
    P4DIR     |=  0x40;                           // [P4.6] radio reset: output
    
    leds_init();
    debugpins_init();
    spi_init();
    radio_init();
    radiotimer_init();
   
    // add callback functions radio
    radio_setOverflowCb(cb_radioTimerOverflows);
    radio_setCompareCb(cb_radioTimerCompare);
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);
    
    // enable interrupts
    __bis_SR_register(GIE);
    
    // overflow interupt every 1 second
    radiotimer_setPeriod(SENDING_PERIOD);
    
    // turn on crystal oscillator
    radio_rfOn();
    radio_setFrequency(CHANNEL);
    
    // write short address
    cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR,
                       &cc2420_status,
                       &cc2420_shortadr[0],
                       2);
    // write panId
    cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,
                       &cc2420_status,
                       &cc2420_panid[0],
                       2);
    // write 64-bit ieee address
    cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,
                       &cc2420_status,
                       &cc2420_ieeeadr[0],
                       8);
    
    radio_rxEnable();
    
    while(1){
        if (app_vars.okToSend){
            app_vars.okToSend=0;
            radio_rfOff();
            fr_autoack_loadPacket();
            radio_loadPacket(&app_vars.packet[0],FRAME_LENGTH);
            radio_txEnable();
            radio_txNow();
        } else {
            radio_rxEnable();
        }
    }
    
}

//=========================== prototype =======================================

void     cb_radioTimerOverflows(void){
    app_vars.okToSend = 1;
    leds_error_toggle();
}

void     cb_radioTimerCompare(void){
    return;
}

void     cb_startFrame(PORT_RADIOTIMER_WIDTH timestamp){
    leds_sync_toggle();
}

void     cb_endFrame(PORT_RADIOTIMER_WIDTH timestamp){
    leds_sync_toggle();
}

void     fr_autoack_loadPacket(){
    app_vars.packet[0] = FRAME_CONTROL_BYTE0; // fcf byte0
    app_vars.packet[1] = FRAME_CONTROL_BYTE1; // fcf byte1
    app_vars.packet[2] = app_vars.dsn;        // dsn
    app_vars.dsn++;
    app_vars.packet[3] = 0xff;                // panId, LITTLE_ENDIAN
    app_vars.packet[4] = 0xff;                
    app_vars.packet[5] = 0xaa;                // destAddr, LITTLE_ENDIAN
    app_vars.packet[6] = 0xaa;                
    app_vars.packet[7] = 0x00;                // reserved for crc
    app_vars.packet[8] = 0x00;
}

//=========================== interrupt handlers ==============================

ISR(TIMERB1) {
   debugpins_isr_set();
   if (radiotimer_isr()==KICK_SCHEDULER) {       // radiotimer
      __bic_SR_register_on_exit(CPUOFF);
   }
   debugpins_isr_clr();
}
