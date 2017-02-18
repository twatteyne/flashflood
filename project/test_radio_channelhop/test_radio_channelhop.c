#include "msp430f1611.h"
#include "stdint.h"
#include "radio.h"
#include "cc2420.h"
#include "timer_a.h"
#include "timer_b.h"
#include "spi.h"
#include "eui64.h"

// ========================== define ==========================================

#define SENDER   0x5e
#define RECEIVER 0xc8

#define FERQUENCY_CHANGE_PERIOD 3276 // 100 milisecond @32768hz

#define LED_LIGHT_INIT            P5DIR |=  0x40; // P5.6
#define LED_LIGHT_ON              P5OUT &= ~0x40;
#define LED_LIGHT_OFF             P5OUT |=  0x40;
#define LED_LIGHT_TOGGLE          P5OUT ^=  0x40;

#define LED_ENDFRAME_INIT            P5DIR |=  0x20; // P5.6
#define LED_ENDFRAME_ON              P5OUT &= ~0x20;
#define LED_ENDFRAME_OFF             P5OUT |=  0x20;
#define LED_ENDFRAME_TOGGLE          P5OUT ^=  0x20;

// ========================= variable =========================================

uint8_t ok_to_send;
uint8_t myId;
//uint8_t frame_buffer[9] = {0x61,0x18,0x00,0xca,0xca,0x02,0x02,0x00,0x00};
uint8_t frame_buffer[9] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

// ========================== prototype =======================================

void timera_ccr1_compare_change_freq_cb(uint16_t timestamp);
void timer_b_cb_endFrame(uint16_t timestamp_timerA,uint16_t timestamp_timerB);

// ========================== main ============================================

/**
\brief The program test the frequency setting of radio.
*/
int main(void) {
    uint16_t   delay;
    uint16_t   regFreq = 0;
    uint16_t   regValue;\
    uint8_t    eui64[8];
   
    ok_to_send = 0;
    WDTCTL     =  WDTPW + WDTHOLD;                // disable watchdog timer
   
    DCOCTL     =  DCO0 | DCO1 | DCO2;             // MCLK at 8MHz
    BCSCTL1    =  RSEL0 | RSEL1 | RSEL2;          // MCLK at 8MHz
   
    LED_LIGHT_INIT  
    LED_ENDFRAME_INIT
    LED_LIGHT_OFF
    LED_ENDFRAME_OFF
     
    // get my EUI64
    eui64_get(&eui64[0]);
    myId = eui64[7];  
    
    timer_a_init();
    timer_a_setCompareCCR1andReturnTBRcb(timera_ccr1_compare_change_freq_cb);
    
    timer_b_init();
    timer_b_setEndFrameCb(timer_b_cb_endFrame);
    
    
    // for calculating subticks, cancel it later if this is not overflow
    TACCR1  =  TAR+FERQUENCY_CHANGE_PERIOD;
    TACCTL1 =  CCIE;
   
   /**** initialize radio */
    spi_init();
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
    
    // turn on radio osc
    radio_oscillatorOn();

    // load the packet 
    radio_loadPacket(&frame_buffer[0],9);
    
    // enable global interrupt
    __bis_SR_register(GIE);

//            regFreq = (regFreq+1)%0x03ff;    // 10 bits frequency field
        
    // configure FSCTRL register
    // 15:14 LOCK_THR          01 (recommended, see p 69 of datasheet)
    //    13 CAL_DONE            0 
    //    12 CAL_RUNNING          0
    //    11 LOCK_LENGTH            0
    //    10 LOCK_STATUS             0 
    //   9:0 FREQ                      f requ ency
    //                         0100 000x xxxx xxxx
    //   frequency = 2048 + FREQ;
    regValue = regFreq | 0x4000;
    
    // configuration radio frequency
    P4OUT          &= ~0x04;
    
    // CC2420_FLAG_WRITE | CC2420_FLAG_REG | CC2420_FSCTRL_ADDR;
    U0TXBUF         = 0x18;
    while ((IFG1 & URXIFG0)==0);
    IFG1           &= ~URXIFG0;
    
    // register value High byte
    U0TXBUF         = (uint8_t)((regValue&0xff00)>>8);
    while ((IFG1 & URXIFG0)==0);
    IFG1           &= ~URXIFG0;
    
    // register value Low byte
    U0TXBUF         = (uint8_t)(regValue&0x00ff);
    while ((IFG1 & URXIFG0)==0);
    IFG1           &= ~URXIFG0;
    
    P4OUT          |=  0x04;
        
    if (myId == SENDER){
      while (ok_to_send==0){
        // send TXON strobe 
        P4OUT          &= ~0x04;
        
        U0TXBUF         = CC2420_STXON;
        while ((IFG1 & URXIFG0)==0);
        IFG1           &= ~URXIFG0;
        
        P4OUT          |=  0x04;
      }
    } else {
        if (myId == RECEIVER){
            radio_rxNow();
        }
    }
}

// ========================== prototype ========================================

void timera_ccr1_compare_change_freq_cb(uint16_t timestamp){
    LED_LIGHT_TOGGLE
    ok_to_send = 1;
    TACCR1  =  TAR+FERQUENCY_CHANGE_PERIOD;
    TACCTL1 =  CCIE;
}

void timer_b_cb_endFrame(uint16_t timestamp_timerA,uint16_t timestamp_timerB){
    uint8_t        rx_for_me;
    
    // determine when I've just receive a packet for me
    rx_for_me = (P1IN & 0x01);
    
    if (rx_for_me==1){
        LED_ENDFRAME_TOGGLE
        // flush rx buffer twice
        P4OUT         &= ~0x04;
        U0TXBUF        =  CC2420_SFLUSHRX;
        while ((IFG1 & URXIFG0)==0);
        IFG1          &= ~URXIFG0;
        U0TXBUF        =  CC2420_SFLUSHRX;
        while ((IFG1 & URXIFG0)==0);
        IFG1          &= ~URXIFG0;
        P4OUT         |=  0x04;
    }
}