#include "msp430f1611.h"
#include "spi.h"
#include "cc2420.h"
//=========================== define ==========================================

#define CHANNEL               26
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
    bool                 f_SFDreceived;
    cc2420_status_t      cc2420_status;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     ft_autoack_radioTimerOverflows(void);
void     ft_autoack_radioTimerCompare(void);
void     ft_autoack_startFrame(PORT_RADIOTIMER_WIDTH timestamp);
void     ft_autoack_endFrame(PORT_RADIOTIMER_WIDTH timestamp);

void     ft_autoack_loadPacket();
// interrupt handlers
kick_scheduler_t   radiotimer_isr(void);

//=========================== main ============================================


int main(void) {
    volatile uint16_t     delay;
    cc2420_FSCTRL_reg_t   cc2420_FSCTRL_reg;
    cc2420_MDMCTRL0_reg_t cc2420_MDMCTRL0_reg;
    cc2420_TXCTRL_reg_t   cc2420_TXCTRL_reg;
    cc2420_RXCTRL1_reg_t  cc2420_RXCTRL1_reg;
    
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
    
    // ==== init led pins
    P5DIR     |=  0x70;                           // P5DIR = 0bx111xxxx for LEDs
    P5OUT     |=  0x70;                           // P2OUT = 0bx111xxxx, all LEDs off
    
    // ==== init debug pins
    P6DIR |=  0x40;      // frame     [P6.6]
    P6DIR |=  0x80;      // slot      [P6.7]
    P2DIR |=  0x08;      // fsm       [P2.3]
    P2DIR |=  0x40;      // task      [P2.6]
    P6DIR |=  0x01;      // isr       [P6.0]
    P3DIR |=  0x20;      // isruarttx [P3.5]
    P3DIR |=  0x10;      // isruartrx [P3.4]
    P6DIR |=  0x02;      // radio     [P6.1]
    
    // ==== use spi bsp driver
    spi_init();
    
    // ==== radio_reset();
    // set radio VREG pin high
    PORT_PIN_RADIO_VREG_HIGH();
    for (delay=0xffff;delay>0;delay--);           // max. VREG start-up time is 0.6ms

    // set radio RESET pin low
    PORT_PIN_RADIO_RESET_LOW();
    for (delay=0xffff;delay>0;delay--);

    // set radio RESET pin high
    PORT_PIN_RADIO_RESET_HIGH();
    for (delay=0xffff;delay>0;delay--);
   
    // disable address recognition
    cc2420_MDMCTRL0_reg.PREAMBLE_LENGTH      = 2; // 3 leading zero's (IEEE802.15.4 compliant)
    cc2420_MDMCTRL0_reg.AUTOACK              = 1; // turn on auto ack
    cc2420_MDMCTRL0_reg.AUTOCRC              = 1; // turn on auto crc
    cc2420_MDMCTRL0_reg.CCA_MODE             = 3;
    cc2420_MDMCTRL0_reg.CCA_HYST             = 2;
    cc2420_MDMCTRL0_reg.ADR_DECODE           = 1; // turn On address recognition
    cc2420_MDMCTRL0_reg.PAN_COORDINATOR      = 0;
    cc2420_MDMCTRL0_reg.RESERVED_FRAME_MODE  = 1; // accept all frame types
    cc2420_MDMCTRL0_reg.reserved_w0          = 0;
    cc2420_spiWriteReg(
      CC2420_MDMCTRL0_ADDR,
      &app_vars.cc2420_status,
      *(uint16_t*)&cc2420_MDMCTRL0_reg
    );

    // speed up time to TX
    cc2420_TXCTRL_reg.PA_LEVEL               = 31;// max. TX power (~0dBm)
    cc2420_TXCTRL_reg.reserved_w1            = 1;
    cc2420_TXCTRL_reg.PA_CURRENT             = 3;
    cc2420_TXCTRL_reg.TXMIX_CURRENT          = 0;
    cc2420_TXCTRL_reg.TXMIX_CAP_ARRAY        = 0;
    cc2420_TXCTRL_reg.TX_TURNAROUND          = 0; // faster STXON->SFD timing (128us)
    cc2420_TXCTRL_reg.TXMIXBUF_CUR           = 2;
    cc2420_spiWriteReg(
      CC2420_TXCTRL_ADDR,
      &app_vars.cc2420_status,
      *(uint16_t*)&cc2420_TXCTRL_reg
    );

    // apply correction recommended in datasheet
    cc2420_RXCTRL1_reg.RXMIX_CURRENT         = 2;
    cc2420_RXCTRL1_reg.RXMIX_VCM             = 1;
    cc2420_RXCTRL1_reg.RXMIX_TAIL            = 1;
    cc2420_RXCTRL1_reg.LNA_CAP_ARRAY         = 1;
    cc2420_RXCTRL1_reg.MED_HGM               = 0;
    cc2420_RXCTRL1_reg.HIGH_HGM              = 1;
    cc2420_RXCTRL1_reg.MED_LOWGAIN           = 0;
    cc2420_RXCTRL1_reg.LOW_LOWGAIN           = 1;
    cc2420_RXCTRL1_reg.RXBPF_MIDCUR          = 0;
    cc2420_RXCTRL1_reg.RXBPF_LOCUR           = 1; // use this setting as per datasheet
    cc2420_RXCTRL1_reg.reserved_w0           = 0;
    cc2420_spiWriteReg(
      CC2420_RXCTRL1_ADDR,
      &app_vars.cc2420_status,
      *(uint16_t*)&cc2420_RXCTRL1_reg
    );
    
    // ==== start radio timer
       // radio's SFD pin connected to P4.1
    P4DIR   &= ~0x02; // input
    P4SEL   |=  0x02; // in CCI1a/B mode

    // CCR0 contains period of counter
    // do not interrupt when counter reaches TBCCR0, but when it resets
    TBCCR0   =  SENDING_PERIOD-1;

    // CCR1 in capture mode
    TBCCTL1  =  CM_3+SCS+CAP+CCIE;
    TBCCR1   =  0;

    // CCR2 in compare mode (disabled for now)
    TBCCTL2  =  0;
    TBCCR2   =  0;

    // start counting
    TBCTL    =  TBIE+TBCLR;                       // interrupt when counter resets
    TBCTL   |=  MC_1+TBSSEL_1;                    // up mode, clocked from ACLK
    
    // ==== enable interrupts
    __bis_SR_register(GIE);
    
    // ==== turn on crystal oscillator
    cc2420_spiStrobe(CC2420_SXOSCON, &app_vars.cc2420_status);
    while (app_vars.cc2420_status.xosc16m_stable==0) {
        cc2420_spiStrobe(CC2420_SNOP, &app_vars.cc2420_status);
    }
    
    // ==== set frequency 
    cc2420_FSCTRL_reg.FREQ         = CHANNEL-11;
    cc2420_FSCTRL_reg.FREQ        *= 5;
    cc2420_FSCTRL_reg.FREQ        += 357;
    cc2420_FSCTRL_reg.LOCK_STATUS  = 0;
    cc2420_FSCTRL_reg.LOCK_LENGTH  = 0;
    cc2420_FSCTRL_reg.CAL_RUNNING  = 0;
    cc2420_FSCTRL_reg.CAL_DONE     = 0;
    cc2420_FSCTRL_reg.LOCK_THR     = 1;

    cc2420_spiWriteReg(
      CC2420_FSCTRL_ADDR,
      &app_vars.cc2420_status,
      *(uint16_t*)&cc2420_FSCTRL_reg
    );
    
    // ==== write short addr, panid and ieee addr
    // write short address
    cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR,
                       &app_vars.cc2420_status,
                       &cc2420_shortadr[0],
                       2);
    // write panId
    cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,
                       &app_vars.cc2420_status,
                       &cc2420_panid[0],
                       2);
    // write 64-bit ieee address
    cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,
                       &app_vars.cc2420_status,
                       &cc2420_ieeeadr[0],
                       8);
    // ==== radio start listening
    // put radio in reception mode
    cc2420_spiStrobe(CC2420_SRXON, &app_vars.cc2420_status);
    cc2420_spiStrobe(CC2420_SFLUSHRX, &app_vars.cc2420_status);

    // busy wait until radio really listening
    while (app_vars.cc2420_status.rssi_valid==0) {
        cc2420_spiStrobe(CC2420_SNOP, &app_vars.cc2420_status);
    }
    
    // ==== main function
    while(1){
        if (app_vars.okToSend){
            app_vars.okToSend=0;
            cc2420_spiStrobe(CC2420_SFLUSHRX, &app_vars.cc2420_status);
            cc2420_spiStrobe(CC2420_SRFOFF, &app_vars.cc2420_status);
            // fill the packet
            ft_autoack_loadPacket();
            // load packet
            cc2420_spiStrobe(CC2420_SFLUSHTX, &app_vars.cc2420_status);
            cc2420_spiWriteFifo(&app_vars.cc2420_status, &app_vars.packet[0], FRAME_LENGTH, CC2420_TXFIFO_ADDR);
            // tx now
            cc2420_spiStrobe(CC2420_STXON, &app_vars.cc2420_status);
        }
    }
    
}

//=========================== prototype =======================================

void     ft_autoack_radioTimerOverflows(void){
    app_vars.okToSend = 1;
    P6OUT ^=  0x80;     // debug_slot_toggle();
    P5OUT ^=  0x10;     // leds_error_toggle();
}

void     ft_autoack_radioTimerCompare(void){
    return;
}

void     ft_autoack_startFrame(PORT_RADIOTIMER_WIDTH timestamp){
    P5OUT ^=  0x40;     //leds_sync_toggle();
}

void     ft_autoack_endFrame(PORT_RADIOTIMER_WIDTH timestamp){
    P5OUT ^=  0x40;     //leds_sync_toggle();
}

void     ft_autoack_loadPacket(){
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
   P6OUT |=  0x01; //debugpins_isr_set();
   if (radiotimer_isr()==KICK_SCHEDULER) {       // radiotimer
      __bic_SR_register_on_exit(CPUOFF);
   }
   P6OUT &= ~0x01; // debugpins_isr_clr();
}

/**
\brief TimerB CCR1-6 interrupt service routine
*/
kick_scheduler_t radiotimer_isr() {
   PORT_RADIOTIMER_WIDTH tbiv_local;
   
   // reading TBIV returns the value of the highest pending interrupt flag
   // and automatically resets that flag. We therefore copy its value to the
   // tbiv_local local variable exactly once. If there is more than one 
   // interrupt pending, we will reenter this function after having just left
   // it.
   tbiv_local = TBIV;
   
   switch (tbiv_local) {
      case 0x0002: // CCR1 fires
         if (TBCCTL1 & CCI) {
             // SFD pin is high: this was the start of a frame
             ft_autoack_startFrame(TBCCR1);
             app_vars.f_SFDreceived = 1;
             // kick the OS
             return KICK_SCHEDULER;
         } else {
             // SFD pin is low: this was the end of a frame
             if (app_vars.f_SFDreceived == 1) {
                 ft_autoack_endFrame(TBCCR1);
                 app_vars.f_SFDreceived = 0;
             }
             TBCCTL1 &= ~COV;
             TBCCTL1 &= ~CCIFG;
             // kick the OS
             return KICK_SCHEDULER;
         }
         break;
      case 0x0004: // CCR2 fires
         ft_autoack_radioTimerCompare();
         // kick the OS
         return KICK_SCHEDULER;
      case 0x0006: // CCR3 fires
         break;
      case 0x0008: // CCR4 fires
         break;
      case 0x000a: // CCR5 fires
         break;
      case 0x000c: // CCR6 fires
         break;
      case 0x000e: // timer overflow
         ft_autoack_radioTimerOverflows();
         // kick the OS
         return KICK_SCHEDULER;
   }
   return DO_NOT_KICK_SCHEDULER;
}
