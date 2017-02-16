#include "timer_a.h"
#include "timer_b.h"
#include "eui64.h"
#include "spi.h"
#include "radio.h"
#include "cc2420.h"
#include "msp430f1611.h"
#include "competition.h"
#include "string.h"
#include "adc_sensor.h"

//=========================== define =========================================

#define SENSING_NODE           0xa0
#define SINK_NODE              0xab

// sink pin toggle p2.3 when receiving data

#define SUBTICK_SCHEDULE        224     // RETRANSMIT_DELAY<<5
#define RETRANSMIT_DELAY          7     // 7@32768Hz = 210us

#define LUX_THRESHOLD           400
#define LUX_HYSTERESIS          100

// txfifo dsn address (0x003)
#define WRITE_TXFIFO_DSN_BYTE0  0x83    //(CC2420_FLAG_RAM | (0x03 & 0x7F)): 0x03 is address byte 0
#define WRITE_TXFIFO_DSN_BYTE1  0x00    //((0x00 >> 1) & 0xC0) | CC2420_FLAG_RAM_WRITE: 0x00 is address byte 1

//=========================== variables =======================================

static  uint8_t cc2420_shortadr_cycle[2] = {0x11,0x11};
static  uint8_t cc2420_panid_cycle[2]    = {0x11,0x11};
static  uint8_t cc2420_ieeeadr_cycle[8]  = {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11};

app_vars_t app_vars;

//=========================== prototypes ======================================

void timer_a_cb_subtickCalculate(uint16_t timestampe);
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
#ifdef PIN_DEBUG
    P5DIR     |=  0x70;                           // P5DIR = 0bx111xxxx for LEDs
    P5OUT     |=  0x70;                           // P2OUT = 0bx111xxxx, all LEDs off
#endif
#ifdef PIN_DEBUG    
    // debugpins
    P6DIR |=  0x40;      // [P6.6]
    P6DIR |=  0x80;      // [P6.7]
    P2DIR |=  0x40;      // [P2.6]
    P3DIR |=  0x20;      // [P3.5]
    P3DIR |=  0x10;      // [P3.4]
#endif
    P2DIR |=  0x08;      // [P2.3] // GIO2
    P2OUT &= ~0x08;      // turn off by default
    
    timer_a_init();
    timer_b_init();
    spi_init();
    adc_sensor_init();
    
    timer_a_setSubtickCalculateCb(timer_a_cb_subtickCalculate);
    timer_a_setCompareCb(timer_a_cb_compare);
    timer_b_setEndFrameCb(timer_b_cb_endFrame);
    
    /**** initialize radio */
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
    // 3 leading zero's (IEEE802.15.4 compliant)
    // turn on auto ack
    // turn on auto crc 
    // turn On address recognition 
    // accept all frame types
    cc2420_spiWriteReg(CC2420_MDMCTRL0_ADDR,&cc2420_status,0x2af2);
    // speed up time to TX: max. TX power (~0dBm), faster STXON->SFD timing (128us)
    cc2420_spiWriteReg(CC2420_TXCTRL_ADDR,&cc2420_status,0x80ff);
    // apply correction recommended in datasheet
    cc2420_spiWriteReg(CC2420_RXCTRL1_ADDR,&cc2420_status,0x2a56);
    
    // enable global interrupt
    __bis_SR_register(GIE);

    /**** write short addr, panid and ieee addr */
    radio_rfOn();
    // write short address
    cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR, &cc2420_status,&cc2420_shortadr_cycle[0], 2);
    // write panId
    cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,&cc2420_status,&cc2420_panid_cycle[0],2);
    // write 64-bit ieee address
    cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,&cc2420_status,&cc2420_ieeeadr_cycle[0],8);

    //fcf is always the same
    app_vars.packetTx[0] = FRAME_CONTROL_BYTE0; // fcf byte0
    app_vars.packetTx[1] = FRAME_CONTROL_BYTE1; // fcf byte1
    for (i=0;i<4;i++){
        app_vars.packetTx[i+3] = 0x11; // all destination is 0x11
    }
    
    radio_setFrequency(CHANNEL);
    radio_loadPacket(app_vars.packetTx,FRAME_LENGTH);
    radio_rxNow();
    
    // get eui address
    eui64_get(&address[0]);
    app_vars.myId = address[7];
    
    if (app_vars.myId==SENSING_NODE){
        TACCR2   =  TAR+LIGHT_SAMPLE_PERIOD;
        TACCTL2  =  CCIE;
    }
    // start subticks calculating timer
    TACCR1  =  TAR+SUBTICK_SCHEDULE;
    TACCTL1 =  CCIE;

    while (1) {
    }
}

//=========================== callbacks =======================================

void timer_a_cb_compare(void) {
    uint8_t iShouldSend;
    if (app_vars.myId==SENSING_NODE){
        
        app_vars.light_reading = adc_sens_read_total_solar();
        // detect light state switches
        if (       app_vars.light_state==0 && (app_vars.light_reading >= (LUX_THRESHOLD + LUX_HYSTERESIS))) {
          // light was just turned on
          app_vars.light_state = 1;
          iShouldSend = 1;
          P2OUT     |= 0x08;    // set light pin
        } else if (app_vars.light_state==1  && (app_vars.light_reading <  (LUX_THRESHOLD - LUX_HYSTERESIS))) {
          // light was just turned off
          app_vars.light_state = 0;
          iShouldSend = 1;
          P2OUT     &= ~0x08; // clean light pin
        } else {
          // light stays in same state
          iShouldSend = 0;
        }
        
        if (iShouldSend){
#ifdef PIN_DEBUG
            P5OUT     ^=  0x10; // toggle red leds
#endif
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
        }
        TACCR2   =  TAR+LIGHT_SAMPLE_PERIOD;
        TACCTL2  =  CCIE;
    }
}

// for calculating sub ticks
void timer_a_cb_subtickCalculate(uint16_t timestamp){
    uint16_t temp;
    if (app_vars.lastTimestamp==0){
        app_vars.subticks = 149*RETRANSMIT_DELAY;
    } else {
        if (timestamp>app_vars.lastTimestamp){
            temp = timestamp-app_vars.lastTimestamp;
        } else {
            temp  = 0xffff-app_vars.lastTimestamp;
            temp += (timestamp+1);
        }
        app_vars.subticks = (temp>>5); // subticks in RETRANSMIT_DELAY ticks
        
        // at 3V, the typical Frequency ranges from 4.4MHz to 5.4MHz,
        // so the theoretical value of subticks ranges from (134~165)
        if (app_vars.subticks<134*RETRANSMIT_DELAY || app_vars.subticks>165*RETRANSMIT_DELAY){
            app_vars.subticks = 149*RETRANSMIT_DELAY;
        }
    }
    // update lastTimestamp
    app_vars.lastTimestamp = timestamp;
    // for calculating subticks, cancel it later if this is not overflow
    TACCR1  =  TACCR1+SUBTICK_SCHEDULE;
    TACCTL1 =  CCIE;
}

void timer_b_cb_endFrame(uint16_t timestamp){
    uint8_t             packet_len;
    uint8_t             rxLightRankDsn;
    uint8_t             neighborDsn;
    uint8_t             rxByte;
    uint8_t             firstByte;
    uint8_t             secondByte;
    uint8_t             neighborRank;
    
    P4OUT  &= ~0x04;
    // read packet length
    U0TXBUF = 0x7f;  // (CC2420_FLAG_READ | CC2420_FLAG_REG | CC2420_RXFIFO_ADDR)
    while ((IFG1 & URXIFG0)==0);
    IFG1   &= ~URXIFG0;
    U0TXBUF = 0x00;
    while ((IFG1 & URXIFG0)==0);
    IFG1   &= ~URXIFG0;
    packet_len  = U0RXBUF;
    
    // receiving 2 bytes FCF, but not read them
    U0TXBUF = 0x00;
    while ((IFG1 & URXIFG0)==0);
    IFG1   &= ~URXIFG0;
    firstByte = U0RXBUF;
    U0TXBUF = 0x00;
    while ((IFG1 & URXIFG0)==0);
    IFG1   &= ~URXIFG0; 
    secondByte = U0RXBUF;
    
    // get DSN
    U0TXBUF = 0x00;
    while ((IFG1 & URXIFG0)==0);
    IFG1   &= ~URXIFG0; 
    rxLightRankDsn  = U0RXBUF;
    
    /**** reading the crc needs sometime, use following code with concern */
    
    // get crc
//    U0TXBUF = 0x00;
//    while ((IFG1 & URXIFG0)==0);
//    IFG1   &= ~URXIFG0; 
//    U0TXBUF = 0x00;
//    while ((IFG1 & URXIFG0)==0);
//    IFG1   &= ~URXIFG0; 
//    rxCrc  = U0RXBUF;
    /* end */
    
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
    
    if (app_vars.myId==SINK_NODE){
        if (packet_len == FRAME_LENGTH){
            if (
                firstByte==FRAME_CONTROL_BYTE0 && 
                secondByte==FRAME_CONTROL_BYTE1
            ){
                neighborDsn = rxLightRankDsn&0x0f;
                if (
                    (neighborDsn>app_vars.currentDsn && neighborDsn-app_vars.currentDsn  <=0x02) ||
                    (neighborDsn<app_vars.currentDsn && 16+neighborDsn-app_vars.currentDsn<=0x02)
                ){
                    // received packet
                    if (rxLightRankDsn&0x80==0x80){
                        P2OUT |= 0x08;
                    } else {
                        P2OUT &= ~0x08;
                    }
                    app_vars.currentDsn = neighborDsn;
                }
            }
        } else {
            if (packet_len == ACK_LENGTH){
                if (
                    firstByte==0x02 && 
                    secondByte==0x00
                ){
                    neighborDsn = rxLightRankDsn&0x0f;
                    if (
                        (neighborDsn>app_vars.currentDsn && neighborDsn-app_vars.currentDsn  <=0x02) ||
                        (neighborDsn<app_vars.currentDsn && 16+neighborDsn-app_vars.currentDsn<=0x02)
                    ){
                        // received packet
                        if (rxLightRankDsn&0x80==0x80){
                            P2OUT |= 0x08;
                        } else {
                            P2OUT &= ~0x08;
                        }
                        app_vars.currentDsn = neighborDsn;
                    }
                }
            }
        }
        return;
    }
    
    // non SINK node; only check on ACK
    if (packet_len ==  ACK_LENGTH){
        neighborRank = (rxLightRankDsn & 0x70)>>4;
        if (app_vars.myRank==0){
            if (app_vars.myId == SENSING_NODE){
                // sensing node never relay 
                return;
            }
            app_vars.myRank = 1+neighborRank;
        } else {
            if (neighborRank>=app_vars.myRank){
                // do not process if receive packet from node with higher rank than me
                return;
            } else {
                app_vars.myRank = 1+neighborRank;
            }
        }
#ifdef PIN_DEBUG
        P3OUT ^= 0x20;
#endif
        // the whole endOfAck process needs around 183us to finish, schedule a little bit more than this (e.g. 210us).
        TBCCR2   =  timestamp+app_vars.subticks;
        TBCCTL2  =  CCIE;
        
        /* sending RXOFF, TXCAL strobe and write rxDsn in TxFiFo in a row */
        P4OUT  &= ~0x04;
        // send RFOFF
        U0TXBUF  = CC2420_SRFOFF;
        while ((IFG1 & URXIFG0)==0);
        IFG1    &= ~URXIFG0;
        // send TXCAL
        U0TXBUF  = CC2420_STXCAL;
        while ((IFG1 & URXIFG0)==0);
        IFG1    &= ~URXIFG0;
        // write dsn to tx fifo RAM 
        U0TXBUF  = WRITE_TXFIFO_DSN_BYTE0; // address[0] 0x03 | CC2420_FLAG_RAM 0x80
        while ((IFG1 & URXIFG0)==0);
        IFG1    &= ~URXIFG0;
        U0TXBUF  = WRITE_TXFIFO_DSN_BYTE1; // address[1] (0x00>>1)&0xc0 | CC2420_FLAG_RAM_WRITE 0x00
        while ((IFG1 & URXIFG0)==0);
        IFG1    &= ~URXIFG0;
        U0TXBUF  = (rxLightRankDsn & 0x8f) | (app_vars.myRank<<4);
        while ((IFG1 & URXIFG0)==0);
        IFG1    &= ~URXIFG0;
        P4OUT   |=  0x04;
        /**** end ****/
        
        do {
            // wait untile calibration stopped
            P4OUT  &= ~0x04;
            U0TXBUF = (0x58); // (CC2420_FLAG_READ | CC2420_FLAG_REG | CC2420_FSCTRL_ADDR )
            while ((IFG1 & URXIFG0)==0);
            IFG1   &= ~URXIFG0;
            U0TXBUF = 0x00;
            while ((IFG1 & URXIFG0)==0);
            IFG1   &= ~URXIFG0;
            rxByte  = U0RXBUF;
            // stop, I only need the MSB
            P4OUT  |=  0x04;
        } while(rxByte & 0x10);
    }
}