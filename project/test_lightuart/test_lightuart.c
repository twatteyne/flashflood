#include "msp430f1611.h"
#include "stdint.h"
#include "adc_sensor.h"

//=========================== variables =======================================

typedef struct {
              uint8_t lightValue_string[5+2];
              uint8_t uart_lastTxByteIndex;
   volatile   uint8_t uartDone;
   volatile   uint8_t uartSendNow;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

//=========================== main ============================================

/**
\brief The program test the energy consumption under LPM4.
*/
int main(void) {
    volatile uint16_t delay;
    uint16_t lightValue;
    
    // disable watchdog timer
    WDTCTL    =  WDTPW + WDTHOLD;
   
    // setup clock speed
    DCOCTL    =  DCO0 | DCO1 | DCO2;             // MCLK at 8MHz
    BCSCTL1   =  RSEL0 | RSEL1 | RSEL2;          // MCLK at 8MHz
    
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
    
    __bis_SR_register(GIE);                      // enable interrupts
    
    for (delay=0xffff;delay!=0;delay--);
    
    adc_init();
    while (1) {
        // read value
        lightValue = adc_read_light();
        
        // format
        app_vars.lightValue_string[0] = '0'+((lightValue/10000)%10);
        app_vars.lightValue_string[1] = '0'+((lightValue/ 1000)%10);
        app_vars.lightValue_string[2] = '0'+((lightValue/  100)%10);
        app_vars.lightValue_string[3] = '0'+((lightValue/   10)%10);
        app_vars.lightValue_string[4] = '0'+((lightValue/    1)%10);
        app_vars.lightValue_string[5] = '\r';
        app_vars.lightValue_string[6] = '\n';
        
        // send string over UART
        app_vars.uartDone              = 0;
        app_vars.uart_lastTxByteIndex  = 0;
        U1TXBUF     = app_vars.lightValue_string[app_vars.uart_lastTxByteIndex];
        while(app_vars.uartDone==0);
    }
}

#pragma vector = USART1TX_VECTOR
__interrupt void USART1TX_ISR (void) {
    app_vars.uart_lastTxByteIndex++;
    if (app_vars.uart_lastTxByteIndex<sizeof(app_vars.lightValue_string)) {
        U1TXBUF     = app_vars.lightValue_string[app_vars.uart_lastTxByteIndex];
    } else {
        app_vars.uartDone = 1;
    }
}

#pragma vector = USART1RX_VECTOR
__interrupt void USART1RX_ISR (void) {
}