#include "msp430f1611.h"
#include "spi.h"
#include "radio.h"
#include "radiotimer.h"
#include "cc2420.h"

//=========================== define ==========================================

static  uint8_t cc2420_shortadr[2] = {0xaa,0xaa};
static  uint8_t cc2420_panid[2]    = {0xff,0xff};
static  uint8_t cc2420_ieeeadr[8]  = {0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};

//=========================== prototypes ======================================

void     cb_radioTimerOverflows(void);
void     cb_radioTimerCompare(void);
void     cb_startFrame(PORT_RADIOTIMER_WIDTH timestamp);
void     cb_endFrame(PORT_RADIOTIMER_WIDTH timestamp);

//=========================== main ============================================


int main(void) {
    uint8_t buffer_shortadr[2];
    uint8_t buffer_panid[2];
    uint8_t buffer_ieeeadr[8];
    cc2420_status_t cc2420_status;
  
    // disable watchdog timer
    WDTCTL     =  WDTPW + WDTHOLD;
   
    // setup clock speed
    DCOCTL    |=  DCO0 | DCO1 | DCO2;             // MCLK at ~8MHz
    BCSCTL1   |=  RSEL0 | RSEL1 | RSEL2;          // MCLK at ~8MHz
                                                 // by default, ACLK from 32kHz XTAL which is running
   
    // initialize pins
    P4DIR     |=  0x20;                           // [P4.5] radio VREG:  output
    P4DIR     |=  0x40;                           // [P4.6] radio reset: output
   
    P5DIR     |=  0x70;                           // P5DIR = 0bx111xxxx for LEDs
    P5OUT     |=  0x70;                           // P2OUT = 0bx111xxxx, all LEDs off
   
    spi_init();
    radiotimer_init();
    radio_init();
   
    // add callback functions radio
    radio_setOverflowCb(cb_radioTimerOverflows);
    radio_setCompareCb(cb_radioTimerCompare);
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);
    
    // turn on crystal oscillator
    radio_rfOn();
    
    // write short address
    cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR,
                       &cc2420_status,
                       &cc2420_shortadr[0],
                       2);
    cc2420_spiReadRam(CC2420_RAM_SHORTADR_ADDR,
                       &cc2420_status,
                       &buffer_shortadr[0],
                       2);
    // write panId
    cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,
                       &cc2420_status,
                       &cc2420_panid[0],
                       2);
    cc2420_spiReadRam(CC2420_RAM_PANID_ADDR,
                       &cc2420_status,
                       &buffer_panid[0],
                       2);
    // write 64-bit ieee address
    cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,
                       &cc2420_status,
                       &cc2420_ieeeadr[0],
                       8);
    cc2420_spiReadRam(CC2420_RAM_IEEEADR_ADDR,
                       &cc2420_status,
                       &buffer_ieeeadr[0],
                       8);
    
    while(1){
    }
}

//=========================== prototype =======================================

void     cb_radioTimerOverflows(void){
    return;
}

void     cb_radioTimerCompare(void){
    return;
}

void     cb_startFrame(PORT_RADIOTIMER_WIDTH timestamp){
    return;
}

void     cb_endFrame(PORT_RADIOTIMER_WIDTH timestamp){
    return;
}