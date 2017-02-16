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

// mote addresses
#ifdef LOCAL_SETUP
    #define ADDR_SENSING_NODE     0xdd

    #define ADDR_HOP1_A_NODE      0x0f
    #define ADDR_HOP1_B_NODE      0x05

    #define ADDR_HOP2_A_NODE      0x16
    #define ADDR_HOP2_B_NODE      0x57

    #define ADDR_HOP3_A_NODE      0x2b
    #define ADDR_HOP3_B_NODE      0x5e

    #define ADDR_HOP4_A_NODE      0xc8
    #define ADDR_HOP4_B_NODE      0xba

    #define ADDR_SINK_NODE        0x00
#else
    #define ADDR_SENSING_NODE     0xa0
    #define ADDR_SINK_NODE        0xab
#endif

// sensing

#define LIGHT_SAMPLE_PERIOD       100 // @32kHz, 100=3ms
#define LUX_THRESHOLD             400
#define LUX_HYSTERESIS            100

// sink pin toggle p2.3 when receiving data

#define SUBTICK_SCHEDULE          224   // RETRANSMIT_DELAY<<5
#define RETRANSMIT_DELAY          7     // 7@32768Hz = 210us

// txfifo dsn address (0x003)
#define WRITE_TXFIFO_DSN_BYTE0    0x83    //(CC2420_FLAG_RAM | (0x03 & 0x7F)): 0x03 is address byte 0
#define WRITE_TXFIFO_DSN_BYTE1    0x00    //((0x00 >> 1) & 0xC0) | CC2420_FLAG_RAM_WRITE: 0x00 is address byte 1

//==== frame content
#define FRAME_CONTROL_BYTE0       0x61 // 0b0110 0001  |bit6: panId compressed|bit5: AR set|bit4: no frame pending|bit3: sec disable|bit0-2: frame type,data|
#define FRAME_CONTROL_BYTE1       0x18 // 0b0001 1000  |bit14-15: src addr is elided|bit12-13:frame version, may not useful|bit10-11:16-bit dest addr|
#define FRAME_LENGTH_DATA         (2+1+2+2+2) // 2B FCF, 1B DSN, 2B dest panId, 2B dest address, 2B CRC
#define FRAME_LENGTH_ACK          (2+1+2)     // 2B FCF, 1B DSN,                                 2B CRC (per CC2420 datasheet, Figure 23)

#define CHANNEL                   26

//=========================== statics =========================================

//=========================== variables =======================================

typedef struct {
    uint8_t             rxpk_crc;
    uint8_t             packetTx[FRAME_LENGTH_DATA];
    
    uint8_t             myId;
    uint8_t             currentDsn;
    uint8_t             myHop;
#ifdef LOCAL_SETUP
    uint8_t             myRfId;
#endif
    uint16_t            subticks;
    uint16_t            lastTimestamp;
    
    uint8_t             light_state;
    uint16_t            light_reading;
    
    uint8_t             cc2420_shortadr[2];
    uint8_t             cc2420_panid[2];
    uint8_t             cc2420_ieeeadr[16];
} app_vars_t;

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
    uint8_t             i;
    uint8_t             address[8];
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
#ifdef LOCAL_SETUP
    P5DIR     |=  0x70;                           // P5DIR = 0bx111xxxx for LEDs
    P5OUT     |=  0x70;                           // P2OUT = 0bx111xxxx, all LEDs off
#endif
#ifdef LOCAL_SETUP    
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
        
    // get eui address
    eui64_get(&address[0]);
    app_vars.myId = address[7];
    
    timer_a_setCompareCCR1andReturnTBRcb(timer_a_cb_subtickCalculate);
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
    
    // configure TXCTRL register
    // 15:14 TXMIXBUF_CUR          10
    //    13 TX_TURNAROUND           0
    // 12:11 TXMIX_CAP_ARRAY          0 0
    //  10:9 TXMIX_CURRENT               00
    //   8:6 PA_CURRENT                    0 11
    //     5 reserved                          1
    //   4:0 PA_LEVEL                           1 1111
    //                             1000 0000 1111 1111
    //                                8    0    f    f
    cc2420_spiWriteReg(CC2420_TXCTRL_ADDR,  &cc2420_status,0x80ff);
    
    
    // apply correction recommended in datasheet
    
    // configure RXCTRL1 register
    // 15:14 reserved          00
    //    13 RXBPF_LOCUR         1 (recommended, see p 68 of datasheet)
    //    12 RXBPF_MIDCUR         0
    //    11 LOW_LOWGAIN            1
    //    10 MED_LOWGAIN             0
    //     9 HIGH_HGM                 1
    //     8 MED_HGM                   0
    //   7:6 LNA_CAP_ARRAY               01
    //   5:4 RXMIX_TAIL                    01
    //   3:2 RXMIX_VCM                        01
    //   1:0 RXMIX_CURRENT                      10
    //                         0010 1010 0101 0110
    //                            2    a    5    6
    cc2420_spiWriteReg(CC2420_RXCTRL1_ADDR, &cc2420_status,0x2a56);
    
    // enable global interrupt
    __bis_SR_register(GIE);
    
    //==== configure radio
    
    // per datasheet Section 13.5, "the crystal oscillator must be running when accessing the RAM."
    radio_oscillatorOn();
    
#ifdef LOCAL_SETUP 
    switch(app_vars.myId){
    case ADDR_SENSING_NODE:
        app_vars.myHop = 0;
        app_vars.myRfId = 0x11;
      break;
    case ADDR_HOP1_A_NODE:
    case ADDR_HOP1_B_NODE:
        app_vars.myHop = 1;
        app_vars.myRfId = 0x22;
        break;
    case ADDR_HOP2_A_NODE:
    case ADDR_HOP2_B_NODE:
        app_vars.myHop = 2;
        app_vars.myRfId = 0x33;
        break;
    case ADDR_HOP3_A_NODE:
    case ADDR_HOP3_B_NODE:
        app_vars.myHop = 3;
        app_vars.myRfId = 0x44;
        break;
    case ADDR_HOP4_A_NODE:
    case ADDR_HOP4_B_NODE:
        app_vars.myHop = 4;
        app_vars.myRfId = 0x55;
        break;
    case ADDR_SINK_NODE:
        app_vars.myHop = 5;
        app_vars.myRfId = 0x66;
        break;
    default:
        break;
    }
    for (i=0;i<2;i++){
        app_vars.cc2420_shortadr[i] = app_vars.myRfId;
        app_vars.cc2420_panid[i]    = app_vars.myRfId;
    }
    for (i=0;i<8;i++){
        app_vars.cc2420_ieeeadr[i]  = app_vars.myRfId;
    }
#else
    for (i=0;i<2;i++){
        app_vars.cc2420_shortadr[i] = 0x11;
        app_vars.cc2420_panid[i]    = 0x11;
    }
    for (i=0;i<8;i++){
        app_vars.cc2420_ieeeadr[i]  = 0x11;
    }
#endif
    // configure radio's short address
    cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR, &cc2420_status,&app_vars.cc2420_shortadr[0], 2);
    
    // configure radio's PANID
    cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,&cc2420_status,&app_vars.cc2420_panid[0],2);
    
    // configure radio's EUI64
    cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,&cc2420_status,&app_vars.cc2420_ieeeadr[0],8);
    
    //==== create packet to transmit
    
    // fcf is always the same
    app_vars.packetTx[0] = FRAME_CONTROL_BYTE0; // fcf byte0
    app_vars.packetTx[1] = FRAME_CONTROL_BYTE1; // fcf byte1
    for (i=0;i<4;i++){
#ifdef LOCAL_SETUP
        app_vars.packetTx[i+3] = app_vars.myRfId+0x11; // 
#else 
        app_vars.packetTx[i+3] = 0x11; // all destination is 0x11
#endif
    }
    
    radio_setFrequency(CHANNEL);
    radio_loadPacket(app_vars.packetTx,FRAME_LENGTH_DATA);
    radio_rxNow();
    
    if (app_vars.myId==ADDR_SENSING_NODE){
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
    if (app_vars.myId==ADDR_SENSING_NODE){
        
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
#ifdef LOCAL_SETUP
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
#ifdef LOCAL_SETUP
            U0TXBUF  = app_vars.currentDsn | (app_vars.myHop+2)<<4 | (app_vars.light_state<<7) ; // bit7, light on/off, bit 6-4, hop (+2), bit 3-0 dsn
#else
            U0TXBUF  = app_vars.currentDsn | (app_vars.light_state<<7); // bit7, light on/off, bit 6-4, hop (0), bit 3-0 dsn
#endif
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
    uint8_t             rxHopDsn;
    uint8_t             neighborDsn;
    uint8_t             rxByte;
    uint8_t             firstByte;
    uint8_t             secondByte;
    uint8_t             neighborHop;
    
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
    rxHopDsn  = U0RXBUF;
    
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
    
    if (app_vars.myId==ADDR_SINK_NODE){
        if (packet_len == FRAME_LENGTH_DATA){
            if (
                firstByte==FRAME_CONTROL_BYTE0 && 
                secondByte==FRAME_CONTROL_BYTE1
            ){
                neighborDsn = rxHopDsn&0x0f;
                if (
                    (neighborDsn>app_vars.currentDsn && neighborDsn-app_vars.currentDsn  <=0x02) ||
                    (neighborDsn<app_vars.currentDsn && 16+neighborDsn-app_vars.currentDsn<=0x02)
                ){
                    // received packet
                    if (rxHopDsn&0x80==0x80){
                        P2OUT |= 0x08;
                    } else {
                        P2OUT &= ~0x08;
                    }
                    app_vars.currentDsn = neighborDsn;
                }
            }
        } else {
            if (packet_len == FRAME_LENGTH_ACK){
                if (
                    firstByte==0x02 && 
                    secondByte==0x00
                ){
                    neighborDsn = rxHopDsn&0x0f;
                    if (
                        (neighborDsn>app_vars.currentDsn && neighborDsn-app_vars.currentDsn  <=0x02) ||
                        (neighborDsn<app_vars.currentDsn && 16+neighborDsn-app_vars.currentDsn<=0x02)
                    ){
                        // received packet
                        if (rxHopDsn&0x80==0x80){
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
    if (packet_len ==  FRAME_LENGTH_ACK){
#ifdef LOCAL_SETUP
        neighborHop = (rxHopDsn & 0x70)>>4;
        if (app_vars.myHop==0 || app_vars.myHop==1){
            return;
        } else {
            if (app_vars.myHop!=neighborHop){
                return;
            }
        }
#else
        neighborHop = (rxHopDsn & 0x70)>>4;
        if (app_vars.myHop==0){
            if (app_vars.myId == ADDR_SENSING_NODE){
                // sensing node never relays
                return;
            }
            app_vars.myHop = 1+neighborHop;
        } else {
            if (neighborHop>=app_vars.myHop){
                // do not process if receive packet from node with higher hop than me
                return;
            } else {
                app_vars.myHop = 1+neighborHop;
            }
        }
#endif
#ifdef LOCAL_SETUP
        P3OUT ^= 0x20;
#endif
        // update pin staus
        if (rxHopDsn&0x80==0x80){
            P2OUT |= 0x08;
        } else {
            P2OUT &= ~0x08;
        }
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
#ifdef LOCAL_SETUP
        U0TXBUF  = (rxHopDsn & 0x8f) | ((app_vars.myHop+2)<<4);
#else
        U0TXBUF  = (rxHopDsn & 0x8f) | (app_vars.myHop<<4);
#endif
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
    } else {
#ifdef LOCAL_SETUP
        // no need on local setup
#else   
        if (packet_len ==  FRAME_LENGTH_DATA){
            // update myHop
            neighborHop = (rxHopDsn & 0x70)>>4;
            if (app_vars.myHop==0){
                if (app_vars.myId == ADDR_SENSING_NODE){
                    // sensing node never relays
                    return;
                }
                app_vars.myHop = 1+neighborHop;
            } else {
                if (neighborHop>=app_vars.myHop){
                    // do not process if receive packet from node with higher hop than me
                    return;
                } else {
                    app_vars.myHop = 1+neighborHop;
                }
            }
            // update pin staus
            if (rxHopDsn&0x80==0x80){
                P2OUT |= 0x08;
            } else {
                P2OUT &= ~0x08;
            }
        }
#endif
    }
}