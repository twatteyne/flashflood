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
    #define ADDR_HOP1_B_NODE      0x16

    #define ADDR_HOP2_A_NODE      0x2b
    #define ADDR_HOP2_B_NODE      0x57

    #define ADDR_SINK_NODE        0x05
    #define PANID                 0xc0ca
#else
    #define ADDR_SENSING_NODE     0xa0
    #define ADDR_SINK_NODE        0xab
    #define PANID                 0xc01a
#endif

// light sensor
#define LIGHT_SAMPLE_PERIOD       655  // @32kHz, 655=20.00ms
#define ONE_HOP_LATENCY            42  // @32kHz,  42= 1.27ms
#define RADIO_STARTUP_DURATION     30  // @32kHz,  30= 1.00ms

#ifdef LOCAL_SETUP
    #define SENSING_RADIO_OSC_SCHEDULE_TO_TURNON_OFFSET 29+5 // turn on oscillator a little later than hop 1, so hop 1 could hear the packet
    #define HOP1_RADIO_OSC_SCHEDULE_TO_TURNON_OFFSET    29   //  @32kHz, 29=864us 
    #define HOP2_RADIO_OSC_SCHEDULE_TO_TURNON_OFFSET    52   //  @32kHz, 29=1.58ms
#else
    #define SENSING_RADIO_OSC_SCHEDULE_TO_TURNON_OFFSET 34   // should be the same with local setup
#endif

#define LIGHT_THRESHOLD           400
#define LIGHT_HYSTERESIS          100

#define RETRANSMIT_DELAY_TICKS    7                             // 7@32768Hz = 210us, between end of ACK and start of DATA
#define CALIBRATION_PERIOD_TICKS  (RETRANSMIT_DELAY_TICKS<<5)   // RETRANSMIT_DELAY<<5

// txfifo dsn address (0x003)
#define WRITE_TXFIFO_DSN_BYTE0    0x83    //(CC2420_FLAG_RAM | (0x03 & 0x7F)): 0x03 is address byte 0
#define WRITE_TXFIFO_DSN_BYTE1    0x00    //((0x00 >> 1) & 0xC0) | CC2420_FLAG_RAM_WRITE: 0x00 is address byte 1

// frame content
#define FRAME_DATA_LEN            (2+1+2+2+2) // 2B FCF, 1B DSN, 2B dest panId, 2B dest address, 2B CRC
#define FRAME_DATA_FCF0           0x61 // 0b0110 0001  |bit6: panId compressed|bit5: AR set|bit4: no frame pending|bit3: sec disable|bit0-2: frame type,data|
#define FRAME_DATA_FCF1           0x18 // 0b0001 1000  |bit14-15: src addr is elided|bit12-13:frame version, may not useful|bit10-11:16-bit dest addr|
#define FRAME_ACK_LEN             (2+1+2)     // 2B FCF, 1B DSN,                                 2B CRC (per CC2420 datasheet, Figure 23)
#define FRAME_ACK_FCF0            0x02
#define FRAME_ACK_FCF1            0x00

// LEDs
#define LED_LIGHT_INIT            P5DIR |=  0x40; // P5.6
#define LED_LIGHT_ON              P5OUT &= ~0x40;
#define LED_LIGHT_OFF             P5OUT |=  0x40;

//debugpins
#define DEBUGPIN_LIGHT_INIT       P2DIR |=  0x08; // P2.3
#define DEBUGPIN_LIGHT_HIGH       P2OUT |=  0x08;
#define DEBUGPIN_LIGHT_LOW        P2OUT &= ~0x08;
#define DEBUGPIN_RADIO_INIT       P6DIR |=  0x80; // P6.7
#define DEBUGPIN_RADIO_HIGH       P6OUT |=  0x80;
#define DEBUGPIN_RADIO_LOW        P6OUT &= ~0x80;
#define DEBUGPIN_RXFORME_INIT     P2DIR |=  0x40; // P2.6
#define DEBUGPIN_RXFORME_HIGH     P2OUT |=  0x40;
#define DEBUGPIN_RXFORME_LOW      P2OUT &= ~0x40;

//=========================== statics =========================================

//=========================== variables =======================================

typedef struct {
    uint8_t             dataFrameTx[FRAME_DATA_LEN];
    
    uint8_t             myId;
    uint8_t             fTurnOSCOffAtNextEndOfFrame;
#ifdef UART_HOP
    uint8_t             fSomethingToPrint;
    uint8_t             dsnToPrint;
    uint8_t             bufferToPrint[5];
    uint8_t             nextIndexToPrint;
#endif
    uint8_t             my_addr;
    uint8_t             current_seq;
    uint16_t            retransmitDelaySubticks;
    uint16_t            lastTimestamp;
    
    uint8_t             light_state;
    uint16_t            light_reading;
    
    uint8_t             cc2420_shortadr[2];
    uint8_t             cc2420_panid[2];
    uint8_t             cc2420_ieeeadr[16];
    uint8_t             hopping_sequence[16];
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void timera_ccr2_compare_cb(void);
void timera_ccr1_compare_get_tbr_cb(uint16_t timestamp);
void timer_b_cb_endFrame(uint16_t timestamp_timerA, uint16_t timestamp_timerB);
#ifdef UART_HOP
void formatStringToPrint();
#endif

//=========================== main ============================================

int main(void) {
    uint8_t             i;
    uint8_t             eui64[8];
    volatile uint16_t   delay;
    cc2420_status_t     cc2420_status;
    uint16_t            dest;
    
    memset(&app_vars,0,sizeof(app_vars_t));
    memset(&eui64[0],0,8);
    
    app_vars.hopping_sequence[0x00] = 15;
    app_vars.hopping_sequence[0x01] = 20;
    app_vars.hopping_sequence[0x02] = 25;
    app_vars.hopping_sequence[0x03] = 26; //===
    app_vars.hopping_sequence[0x04] = 15;
    app_vars.hopping_sequence[0x05] = 20;
    app_vars.hopping_sequence[0x06] = 25;
    app_vars.hopping_sequence[0x07] = 26; //===
    app_vars.hopping_sequence[0x08] = 15;
    app_vars.hopping_sequence[0x09] = 20;
    app_vars.hopping_sequence[0x0a] = 25;
    app_vars.hopping_sequence[0x0b] = 26; //===
    app_vars.hopping_sequence[0x0c] = 15;
    app_vars.hopping_sequence[0x0d] = 20;
    app_vars.hopping_sequence[0x0e] = 25;
    app_vars.hopping_sequence[0x0f] = 26; //===
    
    //===== fire up board
    
    // disable watchdog timer
    WDTCTL     =  WDTPW + WDTHOLD;
    
    // setup clock speed
    DCOCTL    |=  DCO0 | DCO1 | DCO2;            // MCLK at ~8MHz
    BCSCTL1   |=  RSEL0 | RSEL1 | RSEL2;         // MCLK at ~8MHz
                                                 // by default, ACLK from 32kHz XTAL which is running
    
    //===== my identifiers
    
    // get my EUI64
    eui64_get(&eui64[0]);
    app_vars.myId = eui64[7];
    
#ifdef LOCAL_SETUP
    switch(app_vars.myId){
        case ADDR_SENSING_NODE:
            app_vars.my_addr  = 1;
            break;
        case ADDR_HOP1_A_NODE:
        case ADDR_HOP1_B_NODE:
            app_vars.my_addr  = 2;
            break;
        case ADDR_HOP2_A_NODE:
        case ADDR_HOP2_B_NODE:
            app_vars.my_addr  = 3;
            break;
        case ADDR_SINK_NODE:
            app_vars.my_addr  = 4;
            break;
        default:
            break;
    }
#else
    app_vars.my_addr  = 0x11;
#endif
    for (i=0;i<2;i++){
        app_vars.cc2420_shortadr[i] = app_vars.my_addr;
    }
    for (i=0;i<8;i++){
        app_vars.cc2420_ieeeadr[i]  = app_vars.my_addr;
    }
    app_vars.cc2420_panid[0]        = (uint8_t)((uint16_t)(PANID & 0x00ff)>>0);
    app_vars.cc2420_panid[1]        = (uint8_t)((uint16_t)(PANID & 0xff00)>>8);
    
    //===== initialize peripherals
    
#ifdef ENABLE_LEDS
    // LEDs
    LED_LIGHT_INIT;
    LED_LIGHT_OFF;
#endif
    
#ifdef ENABLE_DEBUGPINS
    // debugpins
    P3DIR    |=  0x10;                           // [P3.4] timerAisr
    P6DIR    |=  0x40;                           // [P6.6] timerBisr
    DEBUGPIN_RXFORME_INIT;
    P3DIR    |=  0x20;                           // [P3.5] sfd
    DEBUGPIN_RADIO_INIT;
#endif
    
#ifdef UART_HOP
    // setup UART (115200 baud)
    P3SEL     =  0xC0;                           // P3.6,7 = UART1TX/RX
    ME2      |=  UTXE1 + URXE1;                  // enable UART1 TX/RX
    UCTL1    |=  CHAR;                           // 8-bit character
    UTCTL1   |=  SSEL1;                          // clocking from SMCLK
    UBR01     =  41;                             // 4.8MHz/115200 - 41.66
    UBR11     =  0x00;                           //
    UMCTL1    =  0x4A;                           // modulation
    UCTL1    &= ~SWRST;                          // clear UART1 reset bit
    IE2      |=  UTXIE1;                         // enable UART1 TX interrupt
#endif

    // light pin
#ifdef LIGHTPIN_ALLMOTES
    if (1) {
#else
    if (app_vars.myId==ADDR_SINK_NODE) {
#endif
        DEBUGPIN_LIGHT_INIT;
        DEBUGPIN_LIGHT_LOW;
    }
    
    // Timer A
    timer_a_init();
    timer_a_setCompareCCR1andReturnTBRcb(timera_ccr1_compare_get_tbr_cb); // calibrate
    timer_a_setCompareCCR2Cb(timera_ccr2_compare_cb); // trigger sensor reading
    // arm CCR1 (calibration)
    TACCR1         =  TAR+CALIBRATION_PERIOD_TICKS;
    TACCTL1        =  CCIE;
    // arm CCR2 (light sensor sampling)
    if (app_vars.myId==ADDR_SENSING_NODE){
        TACCR2     =  TAR+LIGHT_SAMPLE_PERIOD;
        TACCTL2    =  CCIE;
    }
    
    // Timer B
    timer_b_init();
    timer_b_setEndFrameCb(timer_b_cb_endFrame);
    
    // ADC
    if (app_vars.myId==ADDR_SENSING_NODE) {
        adc_init();
    }
    
    //==== switch radio on
    
    // SPI
    spi_init();
    // initialize pins
    P4DIR     |=  0x20;                           // [P4.5] radio VREG:  output
    P4DIR     |=  0x40;                           // [P4.6] radio reset: output
    // set radio VREG pin high
    P4OUT     |=  0x20;
    for (delay=0xffff;delay>0;delay--);           // max. VREG start-up time is 0.6ms
    // set radio RESET pin low
    P4OUT     &= ~0x40; 
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
    
    // per datasheet Section 13.5, "the crystal oscillator must be running when accessing the RAM."
#ifdef ENABLE_DEBUGPINS
    DEBUGPIN_RADIO_HIGH;
#endif
    radio_oscillatorOn();
    
    // configure radio's identifiers
    cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR, &cc2420_status, &app_vars.cc2420_shortadr[0], 2);
    cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,    &cc2420_status, &app_vars.cc2420_panid[0],    2);
    cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,  &cc2420_status, &app_vars.cc2420_ieeeadr[0],  8);
    
    //==== create and load data frame
    
    // create
    app_vars.dataFrameTx[0]       = FRAME_DATA_FCF0;                           // FCF
    app_vars.dataFrameTx[1]       = FRAME_DATA_FCF1;                           //
    app_vars.dataFrameTx[2]       = 0x00;                                      // DSN (to be overwritten)
    app_vars.dataFrameTx[3]       = (uint8_t)((uint16_t)(PANID & 0x00ff)>>0);  // PANID
    app_vars.dataFrameTx[4]       = (uint8_t)((uint16_t)(PANID & 0xff00)>>8);  //
#ifdef LOCAL_SETUP
    dest                          = app_vars.my_addr+1;
#else
    dest                          = app_vars.my_addr;
#endif
    app_vars.dataFrameTx[5]       = dest;                                      // dest
    app_vars.dataFrameTx[6]       = dest;                                      //
    
    // load
    radio_loadPacket(app_vars.dataFrameTx,FRAME_DATA_LEN);
    
    //==== switch radio in RX mode
    radio_setFrequency(app_vars.hopping_sequence[app_vars.current_seq]); // all motes, when booted
    radio_rxNow();
    
    while (1);
}

//=========================== callbacks =======================================

// subticks/tick calibration
void timera_ccr1_compare_get_tbr_cb(uint16_t timestamp){
    uint16_t temp;
    
    if (app_vars.lastTimestamp==0){
        app_vars.retransmitDelaySubticks = 149*RETRANSMIT_DELAY_TICKS;
    } else {
        if (timestamp>app_vars.lastTimestamp){
            temp = timestamp-app_vars.lastTimestamp;
        } else {
            temp  = 0xffff-app_vars.lastTimestamp;
            temp += (timestamp+1);
        }
        app_vars.retransmitDelaySubticks = (temp>>5); // subticks in RETRANSMIT_DELAY ticks
        
        // at 3V, the typical Frequency ranges from 4.4MHz to 5.4MHz,
        // so the theoretical value of retransmitDelaySubticks ranges from (134~165)
        if (app_vars.retransmitDelaySubticks<134*RETRANSMIT_DELAY_TICKS || app_vars.retransmitDelaySubticks>165*RETRANSMIT_DELAY_TICKS){
            app_vars.retransmitDelaySubticks = 149*RETRANSMIT_DELAY_TICKS;
        }
    }
    
    // update lastTimestamp
    app_vars.lastTimestamp = timestamp;
    
    // for calculating subticks, cancel it later if this is not overflow
    TACCR1  =  TACCR1+CALIBRATION_PERIOD_TICKS;
    TACCTL1 =  CCIE;
}

// sample light sensor for sensing node, start cycle for others
void timera_ccr2_compare_cb(void) {
    uint8_t rxByte;  
    
    if (app_vars.myId==ADDR_SENSING_NODE){
        // I'm the sensing node: sample and send
        
        // read light value
        app_vars.light_reading = adc_read_light();
        
        // detect light state switches
        if (       app_vars.light_state==0 && (app_vars.light_reading >= (LIGHT_THRESHOLD + LIGHT_HYSTERESIS))) {
            // light was just turned on
            
            app_vars.light_state  = 1;
#ifdef LIGHTPIN_ALLMOTES
            DEBUGPIN_LIGHT_HIGH; // at sensing node
#endif
#ifdef ENABLE_LEDS
            LED_LIGHT_ON; // at sensing node
#endif
        } else if (app_vars.light_state==1  && (app_vars.light_reading <  (LIGHT_THRESHOLD - LIGHT_HYSTERESIS))) {
            // light was just turned off
            
            app_vars.light_state  = 0;
#ifdef LIGHTPIN_ALLMOTES
            DEBUGPIN_LIGHT_LOW; // at sensing node
#endif
#ifdef ENABLE_LEDS
            LED_LIGHT_OFF; // at sensing node
#endif
        }
        
#ifdef ENABLE_DEBUGPINS
        DEBUGPIN_RADIO_HIGH; // at sending node, after sampling light
#endif
        
        // configure frequency
        radio_setFrequency(app_vars.hopping_sequence[app_vars.current_seq]); // sensing node
        
        // turn on oscillator
        P4OUT          &= ~0x04;
        U0TXBUF         =  CC2420_SXOSCON;
        while ((IFG1 & URXIFG0)==0);
        IFG1           &= ~URXIFG0;
        
        // wait until oscillator is on (status byte returned: bit 6 xosc16m_stable)
        do {
            U0TXBUF     = CC2420_SNOP;
            while ((IFG1 & URXIFG0)==0);
            IFG1       &= ~URXIFG0;
            rxByte      = U0RXBUF;
        } while ((rxByte & 0x40) == 0);
        
        // write DSN to TXFIFO RAM 
        P4OUT          &= ~0x04;
        U0TXBUF         = WRITE_TXFIFO_DSN_BYTE0; // address[0] 0x03 | CC2420_FLAG_RAM 0x80
        while ((IFG1 & URXIFG0)==0);
        IFG1           &= ~URXIFG0;
        U0TXBUF         = WRITE_TXFIFO_DSN_BYTE1; // address[1] (0x00>>1)&0xc0 | CC2420_FLAG_RAM_WRITE 0x00
        while ((IFG1 & URXIFG0)==0);
        IFG1           &= ~URXIFG0;
        //                light                       hop        seq
        U0TXBUF         = (app_vars.light_state<<7) | ((0)<<4) | app_vars.current_seq;
        while ((IFG1 & URXIFG0)==0);
        IFG1           &= ~URXIFG0;
        P4OUT          |=  0x04;
        
        // send TXON strobe 
        P4OUT          &= ~0x04;
        U0TXBUF         = CC2420_STXON;
        while ((IFG1 & URXIFG0)==0);
        IFG1           &= ~URXIFG0;
        P4OUT          |=  0x04;
        
        // schedule to turn off radio when done sending
        app_vars.fTurnOSCOffAtNextEndOfFrame = 1;
        
        // increment sequence number, ready for next active period
        app_vars.current_seq = (app_vars.current_seq+1)%16; // at sensing node
        
        // re-arm TimerA CCR2 (same period)
        TACCR2          =  TACCR2+LIGHT_SAMPLE_PERIOD;
        TACCTL2         =  CCIE;
    
    } else {
        // I'm a normal node (or sink): listen
        
#ifdef ENABLE_DEBUGPINS
        DEBUGPIN_RADIO_HIGH; // at not-sensing node
#endif
        
        // configure frequency
        radio_setFrequency(app_vars.hopping_sequence[app_vars.current_seq]); // normal nodes (and sink)
        
        // turn on oscillator
        P4OUT      &= ~0x04;
        U0TXBUF     = CC2420_SXOSCON;
        while ((IFG1 & URXIFG0)==0);
        IFG1       &= ~URXIFG0;
        // wait until oscillator is on. status byte returned: bit 6 xosc16m_stable
        do {
            U0TXBUF     = CC2420_SNOP;
            while ((IFG1 & URXIFG0)==0);
            IFG1       &= ~URXIFG0;
            rxByte      = U0RXBUF;
        } while ((rxByte & 0x40) == 0);
        
        // change to rx state
        U0TXBUF     = CC2420_SRXON;
        while ((IFG1 & URXIFG0)==0);
        IFG1       &= ~URXIFG0;
        
        // flush rxfifo
        U0TXBUF     = CC2420_SFLUSHRX;
        while ((IFG1 & URXIFG0)==0);
        IFG1       &= ~URXIFG0;
        
        // wait  until radio really listen. status byte returned: bit 1: RSSI_VALID
        do {
            U0TXBUF     = CC2420_SNOP;
            while ((IFG1 & URXIFG0)==0);
            IFG1       &= ~URXIFG0;
            rxByte      = U0RXBUF;
        } while (rxByte & 0x02==0);
        P4OUT      |=  0x04;
        
        // disable Timer A CCR2 (will be rearmed after receiving frame)
        TACCR2   =  0;
        TACCTL2  =  ~CCIE;
    }
}

// done receiving an end-of-frame signal
void timer_b_cb_endFrame(uint16_t timestamp_timerA, uint16_t timestamp_timerB){
    // raw packet received
    uint8_t        rx_for_me;
    uint8_t        rxpkt_len;
    uint8_t        rxpkt_fcf0;
    uint8_t        rxpkt_fcf1;
    uint8_t        rxpkt_dsn;
    // parse DSN
    uint8_t        rx_light;
    uint8_t        rx_hop;
    uint8_t        rx_seq;
    // value of the FSCTRL register (1st byte only)
    uint8_t        reg_FSCTRL_byte0;
    uint16_t       newCompareValue;
    
    // determine when I've just receive a packet for me
    rx_for_me = (P1IN & 0x01);
    
#ifdef ENABLE_DEBUGPINS
    if (rx_for_me==1) {
        DEBUGPIN_RXFORME_HIGH;
        DEBUGPIN_RXFORME_LOW;
    }
#endif
    
    // turn off radio if asked to, then abort
    if (app_vars.fTurnOSCOffAtNextEndOfFrame==1 && rx_for_me==0) {
        
        // turn radio off
        P4OUT          &= ~0x04;
        U0TXBUF         = CC2420_SXOSCOFF;
        while ((IFG1 & URXIFG0)==0);
        IFG1           &= ~URXIFG0;
        P4OUT          |=  0x04;
#ifdef ENABLE_DEBUGPINS
        // debug pins
        DEBUGPIN_RADIO_LOW; // after sending frame
#endif
#ifdef UART_HOP
        // print over UART
        if (app_vars.fSomethingToPrint==1) {
            formatStringToPrint();
            U1TXBUF = app_vars.bufferToPrint[app_vars.nextIndexToPrint];
        }
#endif
        
        // abort
        app_vars.fTurnOSCOffAtNextEndOfFrame=0;
        return;
    }
    
    // abort if not for me
    if (rx_for_me==0) {
        return;
    }
    
    // if I get here, I received a packet for me
    
    // abort if I'm the sensing node, which doesn't care about receiving packets
    if (app_vars.myId==ADDR_SENSING_NODE) {
        return;
    }
    
    //===== read rx packet from radio

    //>>>>> CS low
    P4OUT         &= ~0x04;
    
    // read RX packet length
    U0TXBUF        =  0x7f;  // (CC2420_FLAG_READ | CC2420_FLAG_REG | CC2420_RXFIFO_ADDR)
    while ((IFG1 & URXIFG0)==0);
    IFG1          &= ~URXIFG0;
    U0TXBUF        =  0x00;
    while ((IFG1 & URXIFG0)==0);
    IFG1          &= ~URXIFG0;
    rxpkt_len      =  U0RXBUF;
    
    // read RX packet FCF
    U0TXBUF        =  0x00;
    while ((IFG1 & URXIFG0)==0);
    IFG1          &= ~URXIFG0;
    rxpkt_fcf0     =  U0RXBUF;
    U0TXBUF        =  0x00;
    while ((IFG1 & URXIFG0)==0);
    IFG1          &= ~URXIFG0; 
    rxpkt_fcf1     =  U0RXBUF;
    
    // read RX packet DSN
    U0TXBUF        =  0x00;
    while ((IFG1 & URXIFG0)==0);
    IFG1          &= ~URXIFG0; 
    rxpkt_dsn      =  U0RXBUF;
    
    //>>>>> CS high
    P4OUT         |=  0x04;
    
    // flush rx buffer twice
    P4OUT         &= ~0x04;
    U0TXBUF        =  CC2420_SFLUSHRX;
    while ((IFG1 & URXIFG0)==0);
    IFG1          &= ~URXIFG0;
    U0TXBUF        =  CC2420_SFLUSHRX;
    while ((IFG1 & URXIFG0)==0);
    IFG1          &= ~URXIFG0;
    P4OUT         |=  0x04;
    
    //===== handle packet
    
    // sanity check
    if (
            (rxpkt_len==FRAME_DATA_LEN && rxpkt_fcf0==FRAME_DATA_FCF0 && rxpkt_fcf1==FRAME_DATA_FCF1 && rx_for_me==1) ||
            (rxpkt_len==FRAME_ACK_LEN  && rxpkt_fcf0==FRAME_ACK_FCF0  && rxpkt_fcf1==FRAME_ACK_FCF1  && rx_for_me==1)
        ) {
        // this is a valid packet, go on
    } else {
        // abort, wrong length or FCF contents
        return;
    }
    
    // parse IEEE802.15.4 DSN field
    rx_light       = ((rxpkt_dsn&0x80)>>7);
    rx_hop         = ((rxpkt_dsn&0x70)>>4);
    rx_seq         = ((rxpkt_dsn&0x0f)>>0); 
    
#ifdef UART_HOP
    // store DSN field to print over UART
    app_vars.dsnToPrint = rxpkt_dsn;
    app_vars.fSomethingToPrint = 1;
#endif
                
    // apply light setting
    if (rx_light==1){
#ifdef LIGHTPIN_ALLMOTES
        if (1) {
#else
        if (app_vars.myId==ADDR_SINK_NODE) {
#endif
            DEBUGPIN_LIGHT_HIGH;
        }
#ifdef ENABLE_LEDS
        LED_LIGHT_ON;
#endif
    } else {
#ifdef LIGHTPIN_ALLMOTES
        if (1) {
#else
        if (app_vars.myId==ADDR_SINK_NODE) {
#endif
            DEBUGPIN_LIGHT_LOW;
        }
#ifdef ENABLE_LEDS
        LED_LIGHT_OFF;
#endif
    }
    
    // record seq
    app_vars.current_seq = rx_seq;
    
    //=== decide to relay or not
    
    if        (app_vars.myId==ADDR_SINK_NODE){
        // I'm the sink node
        // I never relay, radio can be turned off
        
        // turn radio off
        P4OUT          &= ~0x04;
        U0TXBUF         = CC2420_SXOSCOFF;
        while ((IFG1 & URXIFG0)==0);
        IFG1           &= ~URXIFG0;
        P4OUT          |=  0x04;
        // debug pins
#ifdef ENABLE_DEBUGPINS
        DEBUGPIN_RADIO_LOW;
#endif
#ifdef UART_HOP
        // print over UART
        if (app_vars.fSomethingToPrint==1) {
            formatStringToPrint();
            U1TXBUF = app_vars.bufferToPrint[app_vars.nextIndexToPrint];
        }
#endif
    } else {
        if (rxpkt_len==FRAME_ACK_LEN){
            // normal mote, received ACK
            // I relay in SW
            
            //===== arm Timer B, it will issue the TXON strobe
            TBCCR2      = timestamp_timerB+app_vars.retransmitDelaySubticks;
            TBCCTL2     = CCIE;
            
            //===== prepare radio to relay
            
            //>>>>> CS low
            P4OUT      &= ~0x04;
            
            // send RFOFF
            U0TXBUF     = CC2420_SRFOFF;
            while ((IFG1 & URXIFG0)==0);
            IFG1       &= ~URXIFG0;
            
            // send TXCAL
            U0TXBUF     = CC2420_STXCAL;
            while ((IFG1 & URXIFG0)==0);
            IFG1       &= ~URXIFG0;
            
            // write DSN to TXFIFO
            U0TXBUF     = WRITE_TXFIFO_DSN_BYTE0;     // address[0] 0x03 | CC2420_FLAG_RAM 0x80
            while ((IFG1 & URXIFG0)==0);
            IFG1       &= ~URXIFG0;
            
            U0TXBUF     = WRITE_TXFIFO_DSN_BYTE1;     // address[1] (0x00>>1)&0xc0 | CC2420_FLAG_RAM_WRITE 0x00
            while ((IFG1 & URXIFG0)==0);
            IFG1       &= ~URXIFG0;

            U0TXBUF     = (rx_light<<7) | ((rx_hop+2)<<4) | rx_seq;
            while ((IFG1 & URXIFG0)==0);
            IFG1       &= ~URXIFG0;
            
            //<<<<< CS high
            P4OUT      |=  0x04;
            
            // wait for radio calibration to be done
            do {
                //>>>>> CS low
                P4OUT  &= ~0x04;
                
                U0TXBUF = 0x58; // (CC2420_FLAG_READ | CC2420_FLAG_REG | CC2420_FSCTRL_ADDR )
                while ((IFG1 & URXIFG0)==0);
                IFG1   &= ~URXIFG0;
                
                U0TXBUF = 0x00;
                while ((IFG1 & URXIFG0)==0);
                IFG1   &= ~URXIFG0;
                
                reg_FSCTRL_byte0  = U0RXBUF;
                
                // no need to read second byte of FSCTRL
                
                //<<<<< CS high
                P4OUT  |=  0x04;
            
            } while(reg_FSCTRL_byte0 & 0x10);
            
            // Timer B will fire and issue the command to send the packet
            // nothing to do here
            
            // schedule to turn off the radio after the SW forward
            app_vars.fTurnOSCOffAtNextEndOfFrame=1;
        } else {
            // normal mote, receiving DATA
            // radio auto-ack feature relays it, I don't have anything to do
            
            // schedule to turn off the radio after the HW forward
            app_vars.fTurnOSCOffAtNextEndOfFrame=1;
        }
    }
    
    // increment sequence number, ready for next active period
    app_vars.current_seq = (app_vars.current_seq+1)%16;
    
    // rearm Timer A CCR2 to wake up at next cycle
    newCompareValue          = timestamp_timerA;
    newCompareValue         += LIGHT_SAMPLE_PERIOD;
    if (rxpkt_len==FRAME_ACK_LEN) {
        // when I receive an ACK frame, I'm at at depth rx_hop+2
        newCompareValue     -= (rx_hop+2)*ONE_HOP_LATENCY;
    } else {
        // when I receive a DATA frame, I'm at at depth rx_hop+1
        newCompareValue     -= (rx_hop+1)*ONE_HOP_LATENCY;
    }
    newCompareValue         -=  RADIO_STARTUP_DURATION;
    
    TACCR2   =  newCompareValue;
    TACCTL2  =  CCIE;
}

#ifdef UART_HOP
void formatStringToPrint() {
    uint8_t rx_seq;
    
    rx_seq         = ((app_vars.dsnToPrint&0x0f)>>0);
  
    app_vars.bufferToPrint[0]     = ' ';
    app_vars.bufferToPrint[1]     = '0'+((app_vars.dsnToPrint&0x80)>>7); // light
    app_vars.bufferToPrint[2]     = '0'+((app_vars.dsnToPrint&0x70)>>4); // hop
    if (rx_seq<10) {
       app_vars.bufferToPrint[3] = '0'+rx_seq;                           // seq
    } else {
       app_vars.bufferToPrint[3] = 'a'+(rx_seq-10);
    }
    
    app_vars.bufferToPrint[4] = '\n';
    app_vars.nextIndexToPrint = 0;
}

#pragma vector = USART1TX_VECTOR
__interrupt void USART1TX_ISR (void) {
    app_vars.nextIndexToPrint++;
    if (app_vars.nextIndexToPrint>sizeof(app_vars.bufferToPrint)) {
    } else {
        U1TXBUF = app_vars.bufferToPrint[app_vars.nextIndexToPrint];
    }
}

#pragma vector = USART1RX_VECTOR
__interrupt void USART1RX_ISR (void) {
}
#endif