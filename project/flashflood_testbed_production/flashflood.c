/*
Companion code to the article
Exploiting Constructive Interference in 802.15.4 Networks: Survey and Hands-On Tutorial

This code runs on TelosB boards only. Refer to the README.md file in this
repository for additional information, including on #defines you can use.

\author Tengfei Chang <poipoi>, February 2017.
\author Thomas Watteyne <thomas.watteyne@inria.fr>, February 2017.
*/

// general
#include "string.h"
// platform
#include "msp430f1611.h"
// drivers
#include "cc2420.h"
#include "radio.h"
#include "eui64.h"
#include "adc_sensor.h"
#include "spi.h"

//=========================== configurations ==================================
/*
This section contains configurations you can change to match your setup.
The rest of this source code can be used untouched.
*/

#define SAMPLE_PERIOD             655  // @32kHz, 655=20ms. Period at which the sensing node senses and sends data.

// mote addresses
#ifdef LOCAL_SETUP
    #ifdef LINEAR_TOPOLOGY
        #define ADDR_SENSING_NODE 0xdd
        #define ADDR_HOP1_NODE    0x0f
        #define ADDR_HOP2_NODE    0x16
        #define ADDR_HOP3_NODE    0x2b
        #define ADDR_HOP4_NODE    0x05
        #define ADDR_SINK_NODE    0x57
    #else
        #define ADDR_SENSING_NODE 0xdd
        #define ADDR_HOP1_A_NODE  0x2b
        #define ADDR_HOP1_B_NODE  0x0f
        #define ADDR_HOP2_A_NODE  0x57
        #define ADDR_HOP2_B_NODE  0x16
        #define ADDR_SINK_NODE    0x05
    #endif
#else
    #define ADDR_SENSING_NODE     0xa0
    #define ADDR_SINK_NODE        0xab
#endif

#define PANID                     0xc0ca

//=========================== defines =========================================

// timing
#define RETRANSMIT_DELAY_TICKS    7                             // 7@32768Hz = 213.5us. For Glossy-type relaying. Between end of ACK and start of DATA frames.
#define CALIBRATION_PERIOD_TICKS  (RETRANSMIT_DELAY_TICKS<<5)   // Duration of a calibration period.
#define BACKTRACK_TUNING          41                            // fine-tuning the backtracking to synchronize sender and receiver. measured

// frequencies
#define FREQUENCY_1 359                                         // 359 = 2407 MHz
#define FREQUENCY_2 379                                         // 379 = 2427 MHz
#define FREQUENCY_3 404                                         // 404 = 2452 MHz
#define FREQUENCY_4 429                                         // 429 = 2477 MHz

// frame contents
#define FRAME_DATA_LEN            (2+1+2+2+2)                   // 2B FCF, 1B DSN, 2B dest panId, 2B dest address, 2B CRC
#define FRAME_DATA_FCF0           0x61                          // 0b0110 0001  |bit6: panId compressed|bit5: AR set|bit4: no frame pending|bit3: sec disable|bit0-2: frame type,data|
#define FRAME_DATA_FCF1           0x18                          // 0b0001 1000  |bit14-15: src addr is elided|bit12-13:frame version, may not useful|bit10-11:16-bit dest addr|
#define FRAME_ACK_LEN             (2+1+2)                       // 2B FCF, 1B DSN, 2B CRC (per CC2420 datasheet, Figure 23)
#define FRAME_ACK_FCF0            0x02
#define FRAME_ACK_FCF1            0x00

// hard-coded commands to write the DSN byte in the TXFIFO
#define WRITE_TXFIFO_DSN_BYTE0    0x83                          // (CC2420_FLAG_RAM | (0x03 & 0x7F)): 0x03 is address byte 0
#define WRITE_TXFIFO_DSN_BYTE1    0x00                          // ((0x00 >> 1) & 0xC0) | CC2420_FLAG_RAM_WRITE: 0x00 is address byte 1

// ADC light sensor settings
#define ADC_LIGHT_THRESHOLD       400                           // raw ADC reading. Threshold between light on/off
#define ADC_LIGHT_HYSTERESIS      100                           //

// light pin
#define LIGHTPIN_INIT             P2DIR |=  0x08; // P2.3
#define LIGHTPIN_HIGH             P2OUT |=  0x08;
#define LIGHTPIN_LOW              P2OUT &= ~0x08;

// LEDs
#ifdef ENABLE_LEDS
#define LED_LIGHT_INIT            P5DIR |=  0x40; // P5.6
#define LED_LIGHT_ON              P5OUT &= ~0x40;
#define LED_LIGHT_OFF             P5OUT |=  0x40;
#else
#define LED_LIGHT_INIT            ;
#define LED_LIGHT_ON              ;
#define LED_LIGHT_OFF             ;
#endif

//debugpins
#ifdef ENABLE_DEBUGPINS
#define DEBUGPIN_TIMERA_INIT      P3DIR |=  0x10; // 3.4
#define DEBUGPIN_TIMERA_HIGH      P3OUT |=  0x10;
#define DEBUGPIN_TIMERA_LOW       P3OUT &= ~0x10;
#define DEBUGPIN_TIMERB_INIT      P6DIR |=  0x40; // 6.6
#define DEBUGPIN_TIMERB_HIGH      P6OUT |=  0x40;
#define DEBUGPIN_TIMERB_LOW       P6OUT &= ~0x40;
#define DEBUGPIN_RXFORME_INIT     P2DIR |=  0x40; // P2.6
#define DEBUGPIN_RXFORME_HIGH     P2OUT |=  0x40;
#define DEBUGPIN_RXFORME_LOW      P2OUT &= ~0x40;
#define DEBUGPIN_SFD_INIT         P3DIR |=  0x20; // P3.5
#define DEBUGPIN_SFD_HIGH         P3OUT |=  0x20;
#define DEBUGPIN_SFD_LOW          P3OUT &= ~0x20;
#define DEBUGPIN_RADIO_INIT       P6DIR |=  0x80; // P6.7
#define DEBUGPIN_RADIO_HIGH       P6OUT |=  0x80;
#define DEBUGPIN_RADIO_LOW        P6OUT &= ~0x80;
#else
#define DEBUGPIN_TIMERA_INIT      ;
#define DEBUGPIN_TIMERA_HIGH      ;
#define DEBUGPIN_TIMERA_LOW       ;
#define DEBUGPIN_TIMERB_INIT      ;
#define DEBUGPIN_TIMERB_HIGH      ;
#define DEBUGPIN_TIMERB_LOW       ;
#define DEBUGPIN_RXFORME_INIT     ;
#define DEBUGPIN_RXFORME_HIGH     ;
#define DEBUGPIN_RXFORME_LOW      ;
#define DEBUGPIN_SFD_INIT         ;
#define DEBUGPIN_SFD_HIGH         ;
#define DEBUGPIN_SFD_LOW          ;
#define DEBUGPIN_RADIO_INIT       ;
#define DEBUGPIN_RADIO_HIGH       ;
#define DEBUGPIN_RADIO_LOW        ;
#endif

//=========================== const ===========================================

// DO NOT EDIT DIRECTLY!
// Generated automatically by the backtracking.py script.
const uint16_t backtracking[16] = {
      0, // hop  0,    0us
     10, // hop  1,  320us
     28, // hop  2,  869us
     52, // hop  3, 1586us
     69, // hop  4, 2136us
     93, // hop  5, 2853us
    111, // hop  6, 3403us
    135, // hop  7, 4121us
    153, // hop  8, 4670us
    176, // hop  9, 5388us
    194, // hop 10, 5937us
    218, // hop 11, 6654us
    236, // hop 12, 7203us
    259, // hop 13, 7922us
    277, // hop 14, 8471us
    301, // hop 15, 9189us
};

#ifdef USE_IEEEE154_FREQUENCIES
// channel hopping on IEEE802.15.4 channels
const uint16_t hopping_sequence[16] = {
    377,           // index 0,  channel 15
    402,           // index 1,  channel 20
    427,           // index 2,  channel 25
    432,           // index 3,  channel 26 ===
    377,           // index 4,  channel 15
    402,           // index 5,  channel 20
    427,           // index 6,  channel 25
    432,           // index 7,  channel 26 ===
    377,           // index 8,  channel 15
    402,           // index 9,  channel 20
    427,           // index 10, channel 25
    432,           // index 11, channel 26 ===
    377,           // index 12, channel 15
    402,           // index 13, channel 20
    427,           // index 14, channel 25
    432            // index 15, channel 26 ===
};
#else
// channel hopping outside IEEE802.15.4 channels
const uint16_t hopping_sequence[16] = {
    FREQUENCY_1,
    FREQUENCY_2,
    FREQUENCY_3,
    FREQUENCY_4,    // ===
    FREQUENCY_1,
    FREQUENCY_2,
    FREQUENCY_3,
    FREQUENCY_4,    // ===
    FREQUENCY_1,
    FREQUENCY_2,
    FREQUENCY_3,
    FREQUENCY_4,    // ===
    FREQUENCY_1,
    FREQUENCY_2,
    FREQUENCY_3,
    FREQUENCY_4     // ===
};
#endif

//=========================== variables =======================================

typedef struct {
    uint8_t        current_seq;
    // fsm
    uint8_t        fTurnOSCOffAtNextEndOfFrame;
    // timing
    uint16_t       retransmitDelaySubticks;
    uint16_t       lastTimestamp;    
    // addressing
    uint8_t        my_board_identifier;
    uint8_t        my_network_addr;
    // adc
    uint8_t        adc_light_state;
    uint16_t       adc_last_reading;
#ifdef UART_HOP
    uint8_t        uart_fSomethingToPrint;
    uint8_t        uart_dsnToPrint;
    uint8_t        uart_bufferToPrint[5];
    uint8_t        uart_nextIndexToPrint;
#endif
    uint8_t        timeraCCR1delayed;
    uint8_t        resetTimeraLater;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void start_active_period(void);
void calibrate_subticks(uint16_t tbr_local);
void end_of_frame_handler(uint16_t tar_local, uint16_t tbr_local);
#ifdef UART_HOP
void formatStringToPrint();
#endif

//=========================== main ============================================

int main(void) {
    uint8_t             my_eui64[8];
    uint8_t             dataFrameTx[FRAME_DATA_LEN];
    cc2420_status_t     cc2420_status;
    uint8_t             cc2420_panid[2];
    uint8_t             cc2420_shortaddr[2];
    volatile uint16_t   delay;
    
    // reset global variables
    memset(&app_vars,0,sizeof(app_vars_t));
    
    //===== fire up board
    
    // disable watchdog timer
    WDTCTL     =  WDTPW + WDTHOLD;
    
    // setup clock speed
    DCOCTL    |=  DCO0  | DCO1  | DCO2;          // MCLK at ~8MHz
    BCSCTL1   |=  RSEL0 | RSEL1 | RSEL2;         // MCLK at ~8MHz
                                                 // by default, ACLK from 32kHz XTAL which is running
    
    //===== set my identifiers
    
    // get my EUI64
    eui64_get(&my_eui64[0]);
    app_vars.my_board_identifier = my_eui64[7];
    
#ifdef LOCAL_SETUP
    #ifdef LINEAR_TOPOLOGY
        switch(app_vars.my_board_identifier){
            case ADDR_SENSING_NODE:
                app_vars.my_network_addr  = 1;
                break;
            case ADDR_HOP1_NODE:
                app_vars.my_network_addr  = 2;
                break;
            case ADDR_HOP2_NODE:
                app_vars.my_network_addr  = 3;
                break;
            case ADDR_HOP3_NODE:
                app_vars.my_network_addr  = 4;
                break;
            case ADDR_HOP4_NODE:
                app_vars.my_network_addr  = 5;
                break;    
            case ADDR_SINK_NODE:
                app_vars.my_network_addr  = 6;
                break;
            default:
                break;
        }
    #else
        switch(app_vars.my_board_identifier){
            case ADDR_SENSING_NODE:
                app_vars.my_network_addr  = 1;
                break;
            case ADDR_HOP1_A_NODE:
            case ADDR_HOP1_B_NODE:
                app_vars.my_network_addr  = 2;
                break;
            case ADDR_HOP2_A_NODE:
            case ADDR_HOP2_B_NODE:
                app_vars.my_network_addr  = 3;
                break;
            case ADDR_SINK_NODE:
                app_vars.my_network_addr  = 4;
                break;
            default:
                break;
        }
    #endif
#else
    app_vars.my_network_addr      = 0x11;
#endif
    cc2420_shortaddr[0]           = 0x00;
    cc2420_shortaddr[1]           = app_vars.my_network_addr;
    cc2420_panid[0]               = (uint8_t)((uint16_t)(PANID & 0x00ff)>>0);
    cc2420_panid[1]               = (uint8_t)((uint16_t)(PANID & 0xff00)>>8);
    
    //===== light pin
    
#ifdef LIGHTPIN_ALLMOTES
    if (1) {
#else
    if (app_vars.my_board_identifier==ADDR_SINK_NODE) {
#endif
        LIGHTPIN_INIT;
        LIGHTPIN_LOW;
    }
    
    //===== LEDs
    
    LED_LIGHT_INIT;
    LED_LIGHT_OFF;
    
    //===== debugpins
    
    DEBUGPIN_TIMERA_INIT;
    DEBUGPIN_TIMERB_INIT
    DEBUGPIN_RXFORME_INIT;
    DEBUGPIN_SFD_INIT
    DEBUGPIN_RADIO_INIT;
    
    //===== UART
    
#ifdef UART_HOP
    // 115200 baud, clocked from SMCLK
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
    
    //===== ADC
    
    if (app_vars.my_board_identifier==ADDR_SENSING_NODE) {
        adc_init();
    }
    
    //===== Timer A
    
    // set CCRA0 registers
    TACCR0   =  0;
    
    // CCR1 (compare mode, triggers calibration)
    TACCR1   =  TAR+CALIBRATION_PERIOD_TICKS;
    TACCTL1  =  CCIE;
    
    // CCR2 (compre mode, triggers cycle)
    if (app_vars.my_board_identifier==ADDR_SENSING_NODE){
        TACCR2     =  TAR+SAMPLE_PERIOD;
        TACCTL2    =  CCIE;
    }
    
    // start timer
    TACTL    =  TAIE+TACLR;                      // interrupt when counter resets
    TACTL   |=  MC_2+TASSEL_1;                   // continue mode, from ACLK
    
    //===== Timer B
    
    // radio's SFD pin connected to P4.1 (timer B capture CCR1)
    P4DIR   &= ~0x02; // input
    P4SEL   |=  0x02; // in CCI1a/B mode
    
    // radio FIFOP pin connected to P1.0
    P1DIR   &= ~0x01; // input
    
    // CCR1 in capture mode
    TBCCTL1  =  CM_3+SCS+CAP+CCIE;
    TBCCR1   =  0;
    
    // CCR2 unused
    TBCCTL2  =  0;
    TBCCR2   =  0;
    
    // start timer
    TBCTL    =  TBIE+TBCLR;                       // interrupt when counter resets
    TBCTL   |=  MC_2+TBSSEL_2;                    // continue mode, clocked from SMCLK
    
    //===== enable global interrupt
    __bis_SR_register(GIE);
    
    //===== radio
    
    //=== switch on
    
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
    
    //=== configure
    
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
    
    // per datasheet Section 13.5, "the crystal oscillator must be running when accessing the RAM."
    DEBUGPIN_RADIO_HIGH;
    radio_oscillatorOn();
    
    // configure radio's identifiers
    cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR, &cc2420_status, &cc2420_shortaddr[0], 2);
    cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,    &cc2420_status, &cc2420_panid[0],     2);
    
    //=== load data frame
    // the data frame stays in the radio's TXFIFO and gets sent over and over
    // after having changed the DSN field directly in the TXFIFO
    
    // create
    dataFrameTx[0]       = FRAME_DATA_FCF0;                           // FCF
    dataFrameTx[1]       = FRAME_DATA_FCF1;                           //
    dataFrameTx[2]       = 0x00;                                      // DSN (to be overwritten)
    dataFrameTx[3]       = (uint8_t)((uint16_t)(PANID & 0x00ff)>>0);  // PANID
    dataFrameTx[4]       = (uint8_t)((uint16_t)(PANID & 0xff00)>>8);  //
    dataFrameTx[5]       = 0x00;                                      // dest
#ifdef LOCAL_SETUP
    dataFrameTx[6]       = app_vars.my_network_addr+1;
#else
    dataFrameTx[6]       = app_vars.my_network_addr;
#endif
    
    // load
    radio_loadPacket(dataFrameTx,FRAME_DATA_LEN);
    
    //=== switch on, RX mode
    radio_setFrequency(hopping_sequence[app_vars.current_seq]); // all motes, when booted
    radio_rxNow();
    
    while (1);
}

//=========================== Timer A =========================================

inline void start_active_period(void) {
    uint8_t rxByte;  
    uint8_t reg_FSCTRL_byte0;
    
    if (app_vars.my_board_identifier==ADDR_SENSING_NODE){
        // I'm the sensing node: sample and send
        
        // read light value
        app_vars.adc_last_reading = adc_read_light();
        
        // detect light state switches
        if (       app_vars.adc_light_state==0 && (app_vars.adc_last_reading >= (ADC_LIGHT_THRESHOLD + ADC_LIGHT_HYSTERESIS))) {
            // light was just turned on
            
            app_vars.adc_light_state  = 1;
#ifdef LIGHTPIN_ALLMOTES
            LIGHTPIN_HIGH;  // at sensing node
#endif
            LED_LIGHT_ON;    // at sensing node
        } else if (app_vars.adc_light_state==1  && (app_vars.adc_last_reading <  (ADC_LIGHT_THRESHOLD - ADC_LIGHT_HYSTERESIS))) {
            // light was just turned off
            
            app_vars.adc_light_state  = 0;
#ifdef LIGHTPIN_ALLMOTES
            LIGHTPIN_LOW; // at sensing node
#endif
            LED_LIGHT_OFF; // at sensing node
        }
        
        DEBUGPIN_RADIO_HIGH; // at sending node, after sampling light
        
        // configure frequency
        radio_setFrequency(hopping_sequence[app_vars.current_seq]); // sensing node
        
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
        U0TXBUF         = (app_vars.adc_light_state<<7) | ((0)<<4) | app_vars.current_seq;
        while ((IFG1 & URXIFG0)==0);
        IFG1           &= ~URXIFG0;
        P4OUT          |=  0x04;
        
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
        
        // send TXON strobe 
        P4OUT          &= ~0x04;
        U0TXBUF         = CC2420_STXON;
        while ((IFG1 & URXIFG0)==0);
        IFG1           &= ~URXIFG0;
        P4OUT          |=  0x04;
        
        // schedule to turn off radio when done sending
        app_vars.fTurnOSCOffAtNextEndOfFrame = 1;
        
        // increment sequence number, ready for next active period
        app_vars.current_seq = (app_vars.current_seq+1)&0x0f; // at sensing node
        
        // re-arm TimerA CCR2 (same period)
        TACCR2          =  TACCR2+SAMPLE_PERIOD;
        TACCTL2         =  CCIE;
    
    } else {
        // I'm a normal node (or sink): listen
        
        DEBUGPIN_RADIO_HIGH; // at not-sensing node
        
        // configure frequency
        radio_setFrequency(hopping_sequence[app_vars.current_seq]); // normal nodes (and sink)
        
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

inline void calibrate_subticks(uint16_t tbr_local){
    uint16_t temp;
    
    // calculate retransmitDelaySubticks
    if (app_vars.lastTimestamp==0){
        app_vars.retransmitDelaySubticks = 149*RETRANSMIT_DELAY_TICKS; // 149 4.8MHz ticks per 32kHz ticks
    } else {
        if (app_vars.timeraCCR1delayed==0){
            temp = tbr_local-app_vars.lastTimestamp;
            app_vars.retransmitDelaySubticks = (temp>>5); // subticks in RETRANSMIT_DELAY ticks
        } else {
            app_vars.timeraCCR1delayed = 0;
        }
    }
    
    // remember lastTimestamp
    app_vars.lastTimestamp = tbr_local;
    
    // re-arm TimerA for next calibration
    TACCR1  =  TACCR1+CALIBRATION_PERIOD_TICKS;
    TACCTL1 =  CCIE;
}

#pragma vector = TIMERA1_VECTOR
__interrupt void TIMERA1_ISR (void) {
    uint16_t   tbr_local;
    uint16_t   taiv_local;
    tbr_local  = TBR;
    taiv_local = TAIV;
    
    // debug pin
    DEBUGPIN_TIMERA_HIGH;
    
    // handle the event
    switch(taiv_local) {
        case 0x0002:
            // CCR1 compare
            calibrate_subticks(tbr_local);
            break;
        case 0x0004:
            // CCR2 compare
            start_active_period();
            break;
        case 0x000a:
            // overflow
            ;
    }
    
    if (TBCCTL1 & CCI){
        app_vars.resetTimeraLater = 1;
    } else {
    // debug pin
    DEBUGPIN_TIMERA_LOW;
    }
    
    // check timera CCR1 interrupt delayed or not
    if (TACCTL1 & CCIFG){
        app_vars.timeraCCR1delayed = 1;
    }
    
    __bic_SR_register_on_exit(CPUOFF);
}

//=========================== Timer B =========================================

inline void end_of_frame_handler(uint16_t tar_local, uint16_t tbr_local){
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
    uint16_t       newCompareValue;
    uint8_t        crcByte;
    uint8_t        i;
    uint8_t        my_hop;
    uint8_t        reg_FSCTRL_byte0;
    
    // determine when I've just receive a packet for me
    rx_for_me = (P1IN & 0x01);
    
    if (rx_for_me==1) {
        DEBUGPIN_RXFORME_HIGH;
        DEBUGPIN_RXFORME_LOW;
    }
    
    // turn off radio if asked to, then abort
    if (app_vars.fTurnOSCOffAtNextEndOfFrame==1 && rx_for_me==0) {
        
        // turn radio off
        P4OUT          &= ~0x04;
        U0TXBUF         = CC2420_SXOSCOFF;
        while ((IFG1 & URXIFG0)==0);
        IFG1           &= ~URXIFG0;
        P4OUT          |=  0x04;
        // debug pins
        DEBUGPIN_RADIO_LOW; // after sending frame
#ifdef UART_HOP
        // print over UART
        if (app_vars.uart_fSomethingToPrint==1) {
            formatStringToPrint();
            U1TXBUF = app_vars.uart_bufferToPrint[app_vars.uart_nextIndexToPrint];
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
    if (app_vars.my_board_identifier==ADDR_SENSING_NODE) {
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
    
    for (i=3;i<rxpkt_len;i++) {
        // read rest of bytes, we only care about last one (contains CRC)
        U0TXBUF    =  0x00;
        while ((IFG1 & URXIFG0)==0);
        IFG1      &= ~URXIFG0; 
        crcByte    =  U0RXBUF;
    }
    
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
    
    // check CRC
    if ((crcByte&0x80)==0) {
        // abort, CRC wrong
        return;
    }
    
    // check packet format
    if (
            (rxpkt_len==FRAME_ACK_LEN  && rxpkt_fcf0==FRAME_ACK_FCF0  && rxpkt_fcf1==FRAME_ACK_FCF1  && rx_for_me==1) ||
            (rxpkt_len==FRAME_DATA_LEN && rxpkt_fcf0==FRAME_DATA_FCF0 && rxpkt_fcf1==FRAME_DATA_FCF1 && rx_for_me==1)
            
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
    
#ifdef LOCAL_SETUP
    // in local only: filter ACKs by hop count as there is no address field
    if (rxpkt_len==FRAME_ACK_LEN && rx_hop!=(app_vars.my_network_addr-3)) {
        return;
    }
#endif
    
#ifdef UART_HOP
    // store DSN field to print over UART
    app_vars.uart_dsnToPrint = rxpkt_dsn;
    app_vars.uart_fSomethingToPrint = 1;
#endif
    
    // apply light setting
    if (rx_light==1){
#ifdef LIGHTPIN_ALLMOTES
        if (1) {
#else
        if (app_vars.my_board_identifier==ADDR_SINK_NODE) {
#endif
            LIGHTPIN_HIGH;
        }
        LED_LIGHT_ON;
    } else {
#ifdef LIGHTPIN_ALLMOTES
        if (1) {
#else
        if (app_vars.my_board_identifier==ADDR_SINK_NODE) {
#endif
            LIGHTPIN_LOW;
        }
        LED_LIGHT_OFF;
    }
    
    // record seq
    app_vars.current_seq = rx_seq;
    
    //=== decide whether to relay
    
    if        (app_vars.my_board_identifier==ADDR_SINK_NODE){
        // I'm the sink node
        // I never relay, radio can be turned off
        
        // turn radio off
        P4OUT          &= ~0x04;
        U0TXBUF         = CC2420_SXOSCOFF;
        while ((IFG1 & URXIFG0)==0);
        IFG1           &= ~URXIFG0;
        P4OUT          |=  0x04;
        // debug pins
        DEBUGPIN_RADIO_LOW;
#ifdef UART_HOP
        // print over UART
        if (app_vars.uart_fSomethingToPrint==1) {
            formatStringToPrint();
            U1TXBUF = app_vars.uart_bufferToPrint[app_vars.uart_nextIndexToPrint];
        }
#endif
    } else {
        // normal mote
        
        if (rxpkt_len==FRAME_ACK_LEN){
            // normal mote, received ACK
            // I relay in SW
            
            //===== arm Timer B, it will issue the TXON strobe
            TBCCR2      = tbr_local+app_vars.retransmitDelaySubticks;
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

            U0TXBUF     = (rx_light<<7) | (((rx_hop+2)&0x07)<<4) | rx_seq;
            while ((IFG1 & URXIFG0)==0);
            IFG1       &= ~URXIFG0;
            
            //<<<<< CS high
            P4OUT      |=  0x04;
            
            /*
            // Per our analysis, waiting for the calibration takes 56us.
            // The calibration is done 217us after the end of frame event.
            // Since the offet is 7 ticks (213.5us), we are confident the radio
            // is calibrated when TimerB expires. So we dont have to actively
            // wait for the calibration to be done.
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
            */
            
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
    app_vars.current_seq     = (app_vars.current_seq+1)&0x0f;
    
    // re-arm Timer A CCR2 to wake up at next cycle
    newCompareValue          = tar_local;
    newCompareValue         += SAMPLE_PERIOD;
    if (rxpkt_len==FRAME_ACK_LEN) {
        my_hop               = rx_hop+2;
    } else {
        my_hop               = rx_hop+1;
    }
    if (my_hop>16) {
        my_hop               = 16;
    }
    newCompareValue         -=  backtracking[my_hop];
    newCompareValue         -=  BACKTRACK_TUNING;
    TACCR2                   =  newCompareValue;
    TACCTL2                  =  CCIE;
}

#pragma vector = TIMERB1_VECTOR
__interrupt void TIMERB1_ISR (void) {
    uint16_t tbiv_local;
    uint16_t tar_local;
    
    tar_local  = TAR;
    tbiv_local = TBIV;
    
    // debugpin
    DEBUGPIN_TIMERB_HIGH;
    
    switch (tbiv_local) {
        case 0x0002:
            // CCR1 capture triggered (SFD pin toggled)
           
            if (TBCCTL1 & CCI) {
                 // SFD pin went high
                 
                 // debugpin
                 DEBUGPIN_SFD_HIGH;
            } else {
                 // SFD pin went low
                 
                 // debugpin
                 DEBUGPIN_SFD_LOW;
                 
                 end_of_frame_handler(tar_local,TBCCR1);
                 
                 TBCCTL1 &= ~COV;
                 TBCCTL1 &= ~CCIFG;
            }
            break;
        case 0x0004:
            // CCR2 compare fired
       
            // send TXON strobe
            P4OUT     &= ~0x04;
            U0TXBUF    =  CC2420_STXON;
            while ((IFG1 & URXIFG0)==0);
            IFG1      &= ~URXIFG0;
            P4OUT     |=  0x04;

            TBCCR2   =  0;
            TBCCTL2 &= ~CCIE;
            break;
    }
    
    // debugpin
    DEBUGPIN_TIMERB_LOW;
    if (app_vars.resetTimeraLater == 1){
    // debug pin
    DEBUGPIN_TIMERA_LOW;
    app_vars.resetTimeraLater = 0;
    }
    
    // check timera CCR1 interrupt delayed or not
    if (TACCTL1 & CCIFG){
        app_vars.timeraCCR1delayed = 1;
    }
    
    __bic_SR_register_on_exit(CPUOFF);
}

//=========================== UART ============================================

#ifdef UART_HOP
void formatStringToPrint() {
    uint8_t rx_seq;
    
    rx_seq         = ((app_vars.uart_dsnToPrint&0x0f)>>0);
  
    app_vars.uart_bufferToPrint[0]     = ' ';
    app_vars.uart_bufferToPrint[1]     = '0'+((app_vars.uart_dsnToPrint&0x80)>>7); // light
    app_vars.uart_bufferToPrint[2]     = '0'+((app_vars.uart_dsnToPrint&0x70)>>4); // hop
    if (rx_seq<10) {
       app_vars.uart_bufferToPrint[3] = '0'+rx_seq;                            // seq
    } else {
       app_vars.uart_bufferToPrint[3] = 'a'+(rx_seq-10);
    }
    
    app_vars.uart_bufferToPrint[4] = '\n';
    app_vars.uart_nextIndexToPrint = 0;
}

#pragma vector = USART1TX_VECTOR
__interrupt void USART1TX_ISR (void) {
    app_vars.uart_nextIndexToPrint++;
    if (app_vars.uart_nextIndexToPrint>sizeof(app_vars.uart_bufferToPrint)) {
    } else {
        U1TXBUF = app_vars.uart_bufferToPrint[app_vars.uart_nextIndexToPrint];
    }
}

#pragma vector = USART1RX_VECTOR
__interrupt void USART1RX_ISR (void) {
}
#endif
