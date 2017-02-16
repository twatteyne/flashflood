// general
#include "string.h"
// platform
#include "msp430f1611.h"
// drivers
#include "adc_sensor.h"
#include "cc2420.h"
#include "eui64.h"
#include "radio.h"
#include "spi.h"
#include "timer_a.h"
#include "timer_b.h"

//=========================== defines =========================================

#define LIGHT_SAMPLE_PERIOD       10000 // 300ms

#define ADDR_SENSING_NODE         0x16

// sink pin toggle p2.3 when receiving data

#define SUBTICK_SCHEDULE          224     // RETRANSMIT_DELAY<<5
#define RETRANSMIT_DELAY          7     // 7@32768Hz = 210us

#define LUX_THRESHOLD             400
#define LUX_HYSTERESIS            100

// txfifo dsn address (0x003)
#define WRITE_TXFIFO_DSN_BYTE0    0x83    //(CC2420_FLAG_RAM | (0x03 & 0x7F)): 0x03 is address byte 0
#define WRITE_TXFIFO_DSN_BYTE1    0x00    //((0x00 >> 1) & 0xC0) | CC2420_FLAG_RAM_WRITE: 0x00 is address byte 1

//==== frame content
#define FRAME_CONTROL_BYTE0       0x61 // 0b0110 0001  |bit6: panId compressed|bit5: AR set|bit4: no frame pending|bit3: sec disable|bit0-2: frame type,data|
#define FRAME_CONTROL_BYTE1       0x18 // 0b0001 1000  |bit14-15: src addr is elided|bit12-13:frame version, may not useful|bit10-11:16-bit dest addr|
#define FRAME_LENGTH_DATA         (2+1+2+2) // 2B FCF, 1B DSN, 2B dest address, 2B CRC
#define FRAME_LENGTH_ACK          (2+1+2)     // 2B FCF, 1B DSN, 2B CRC (per CC2420 datasheet, Figure 23)

#define CHANNEL                   26

//==== mote role
#define SOURCE_ID                 0x00  // no source

#define FIRST_HOP_1               0xba
#define FIRST_HOP_2               0xc8

#define SECOND_HOP_1              0x0f
#define SECOND_HOP_2              0x05

#define THIRD_HOP_1               0x2b
#define THIRD_HOP_2               0x5e

#define FOURTH_HOP_1              0x16
#define FOURTH_HOP_2              0x57

#define DESTINATION_ID            0xdd

//=========================== statics =========================================

static  uint8_t cc2420_shortadr_cycle[2] = {0x11,0x11};
static  uint8_t cc2420_panid_cycle[2]    = {0x11,0x11};
static  uint8_t cc2420_ieeeadr_cycle[8]  = {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11};

//=========================== variables =======================================

typedef struct {
    uint8_t             rxpk_crc;
    cc2420_status_t     cc2420_status;
    uint8_t             packetTx[FRAME_LENGTH_DATA];
    
    uint8_t             myId;
    uint8_t             currentDsn;
    uint8_t             myRank;
    
    uint16_t            subticks;
    uint16_t            lastTimestamp;
    
    uint8_t             light_state;
    uint16_t            light_reading;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void timer_a_cb_compare(void);
void timer_b_cb_endFrame(uint16_t timestamp);

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int main(void) {
    uint8_t i,address[8];
    volatile uint16_t   delay;
    cc2420_status_t     cc2420_status;
    
    memset(&app_vars,0,sizeof(app_vars_t));
    memset(&address[0],0,8);
    
        // disable watchdog timer
    WDTCTL     =  WDTPW + WDTHOLD;
   
    // setup clock speed
    DCOCTL    |=  DCO0 | DCO1 | DCO2;             // MCLK at ~8MHz
    BCSCTL1   |=  RSEL0 | RSEL1 | RSEL2;          // MCLK at ~8MHz
                                                 // by default, ACLK from 32kHz XTAL which is running
    
    timer_a_init();
    timer_b_init();
    spi_init();
    
    timer_a_setCompareCCR2Cb(timer_a_cb_compare);
    timer_b_setEndFrameCb(timer_b_cb_endFrame);
    
    //==== switch radio on
    
    // initialize pins
    P4DIR     |=  0x20;                           // [P4.5] radio VREG:  output
    P4DIR     |=  0x40;                           // [P4.6] radio reset: output
    // set radio VREG pin high
    P4OUT |=  0x20;
    for (delay=0xffff;delay>0;delay--);           // max. VREG start-up time is 0.6ms
    // set radio RESET pin low
    P4OUT &= ~0x40; 
    for (delay=0xffff;delay>0;delay--);
    // set radio RESET pin high
    P4OUT |=  0x40;
    for (delay=0xffff;delay>0;delay--);
    
    //==== configure radio
    
    // configure MDMCTRL0 register
    // 15:14 reserved             00
    //    13 RESERVED_FRAME_MODE    0
    //    12 PAN_COORDINATOR         0
    //    11 ADDR_DECODE               1
    //  10:8 CCA_HYST                   010
    //   7:6 CCA_MODE                       11
    //     5 AUTOCRC                          1
    //     4 AUTOACK                           1         <===
    //   3:0 PREAMBLE_LENGTH                     0010
    //                            0000 1010 1111 0010
    //                               0    a    f    2
    cc2420_spiWriteReg(CC2420_MDMCTRL0_ADDR,&cc2420_status,0x0af2);
    
    // speed up time to TX: max. TX power (~0dBm), faster STXON->SFD timing (128us)
    cc2420_spiWriteReg(CC2420_TXCTRL_ADDR,  &cc2420_status,0x80ff);
    // apply correction recommended in datasheet
    cc2420_spiWriteReg(CC2420_RXCTRL1_ADDR, &cc2420_status,0x2a56);
    
    // enable global interrupt
    __bis_SR_register(GIE);
    
    //==== configure radio
    
    // per datasheet Section 13.5, "the crystal oscillator must be running when accessing the RAM."
    radio_oscillatorOn();
    
    // configure radio's short address
    cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR, &cc2420_status,&cc2420_shortadr_cycle[0], 2);
    
    // configure radio's PANID
    cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,&cc2420_status,&cc2420_panid_cycle[0],2);
    
    // configure radio's EUI64
    cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,&cc2420_status,&cc2420_ieeeadr_cycle[0],8);
    
    //==== create packet to transmit
    
    // fcf is always the same
    app_vars.packetTx[0] = FRAME_CONTROL_BYTE0; // fcf byte0
    app_vars.packetTx[1] = FRAME_CONTROL_BYTE1; // fcf byte1
    for (i=0;i<2;i++){
        app_vars.packetTx[i+3] = 0x11; // destination is 0x11
    }
    
    radio_setFrequency(CHANNEL);
    radio_loadPacket(app_vars.packetTx,FRAME_LENGTH_DATA);
    radio_rxNow();
    
    // get eui address
    eui64_get(&address[0]);
    app_vars.myId = address[7];
    
    if (app_vars.myId==ADDR_SENSING_NODE){
        TACCR2   =  TAR+LIGHT_SAMPLE_PERIOD;
        TACCTL2  =  CCIE;
    }

    while (1) {
    }
}

//=========================== callbacks =======================================

void timer_a_cb_compare(void) {
    if (app_vars.myId==ADDR_SENSING_NODE){
        app_vars.currentDsn = (app_vars.currentDsn+1)%16; // lower 4 bits are dsn
        // write dsn to tx fifo RAM 
        P4OUT   &= ~0x04;
        U0TXBUF  = WRITE_TXFIFO_DSN_BYTE0; // address[0] 0x03 | CC2420_FLAG_RAM 0x80
        while ((IFG1 & URXIFG0)==0);
        IFG1    &= ~URXIFG0;
        U0TXBUF  = WRITE_TXFIFO_DSN_BYTE1; // address[1] (0x00>>1)&0xc0 | CC2420_FLAG_RAM_WRITE 0x00
        while ((IFG1 & URXIFG0)==0);
        IFG1    &= ~URXIFG0;
        U0TXBUF  = app_vars.currentDsn | (app_vars.light_state<<7); // bit7, light on/off, bit 6-4, hop (0), bit 3-0 dsn
        while ((IFG1 & URXIFG0)==0);
        IFG1    &= ~URXIFG0;
        P4OUT   |=  0x04;
        
        // send TXON strobe 
        P4OUT  &= ~0x04;
        // write next byte to TX buffer
        U0TXBUF  = CC2420_STXON;
        // busy wait on the interrupt flag
        while ((IFG1 & URXIFG0)==0);
        // clear the interrupt flag
        IFG1    &= ~URXIFG0;
        // pull highs
        P4OUT   |=  0x04;
            
        TACCR2   =  TAR+LIGHT_SAMPLE_PERIOD;
        TACCTL2  =  CCIE;
    }
}
void timer_b_cb_endFrame(uint16_t timestamp){
    
    P4OUT  &= ~0x04;
    // read packet length
    U0TXBUF = 0x7f;  // (CC2420_FLAG_READ | CC2420_FLAG_REG | CC2420_RXFIFO_ADDR)
    while ((IFG1 & URXIFG0)==0);
    IFG1   &= ~URXIFG0;
    U0TXBUF = 0x00;
    while ((IFG1 & URXIFG0)==0);
    IFG1   &= ~URXIFG0;
    
    // receiving 2 bytes FCF, but not read them
    U0TXBUF = 0x00;
    while ((IFG1 & URXIFG0)==0);
    IFG1   &= ~URXIFG0;
    U0TXBUF = 0x00;
    while ((IFG1 & URXIFG0)==0);
    IFG1   &= ~URXIFG0; 
    
    // get DSN
    U0TXBUF = 0x00;
    while ((IFG1 & URXIFG0)==0);
    IFG1   &= ~URXIFG0;   
    
    P4OUT  |=  0x04;
    
    // flush rx twice
    P4OUT  &= ~0x04;
    U0TXBUF = CC2420_SFLUSHRX;
    while ((IFG1 & URXIFG0)==0);
    IFG1   &= ~URXIFG0;
    U0TXBUF = CC2420_SFLUSHRX;
    while ((IFG1 & URXIFG0)==0);
    IFG1   &= ~URXIFG0;
    P4OUT  |=  0x04;
}