#include "msp430f1611.h"
#include "stdint.h"
#include "adc_sensor.h"

void senduint16(uint16_t val);
void sendchar(uint8_t c);
uint8_t doneSending;

/**
\brief The program test the energy consumption under LPM4.
*/
int main(void) {
    uint16_t lightValue;
    
    // disable watchdog timer
    WDTCTL    =  WDTPW + WDTHOLD;
   
    // setup clock speed
    DCOCTL    =  DCO0 | DCO1 | DCO2;             // MCLK at 8MHz
    BCSCTL1   =  RSEL0 | RSEL1 | RSEL2;          // MCLK at 8MHz
    
    // setup UART
    P3SEL     =  0xC0;                           // P3.6,7 = UART1TX/RX
    //9600 baud, clocked from 32kHz ACLK
    ME2      |=  UTXE1 + URXE1;                  // enable UART1 TX/RX
    UCTL1    |=  CHAR;                           // 8-bit character
    UTCTL1   |=  SSEL0;                          // clocking from ACLK
    UBR01     =  0x03;                           // 32768/9600 = 3.41
    UBR11     =  0x00;                           //
    UMCTL1    =  0x4A;                           // modulation
    UCTL1    &= ~SWRST;                          // clear UART1 reset bit
    IE2      |=  UTXIE1;                         // enable UART1 TX interrupt
    
    __bis_SR_register(GIE);                      // enable interrupts
    
    adc_sensor_init();
    while (1) {
        lightValue = adc_sens_read_total_solar();
        senduint16(lightValue);
    }
}

void senduint16(uint16_t val) {
    sendchar('0'+((val/10000)%10));
    sendchar('0'+((val/ 1000)%10));
    sendchar('0'+((val/  100)%10));
    sendchar('0'+((val/   10)%10));
    sendchar('0'+((val/    1)%10));
    sendchar('\n');
    sendchar('\r');
}

void sendchar(uint8_t c) {
    doneSending = 0;
    U1TXBUF     = c;
    while(doneSending==0);
}

/**
\brief This function is called when the the UART module has received a byte.
*/
#pragma vector = USART1TX_VECTOR
__interrupt void USART1TX_ISR (void)
{
   doneSending = 1;
}