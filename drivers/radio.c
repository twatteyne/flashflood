/**
\brief CC2420-specific definition of the "radio" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#include "radio.h"
#include "cc2420.h"
#include "spi.h"
#include "string.h"

//=========================== defines =========================================

//=========================== variables =======================================

typedef struct {
   cc2420_status_t radioStatusByte;
   radio_state_t   state;
} radio_vars_t;

radio_vars_t radio_vars;

//=========================== public ==========================================

//===== RF admin

void radio_setFrequency(uint8_t frequency) {
   cc2420_FSCTRL_reg_t cc2420_FSCTRL_reg;
   
   // change state
   radio_vars.state = RADIOSTATE_SETTING_FREQUENCY;
   
   cc2420_FSCTRL_reg.FREQ         = frequency-11;
   cc2420_FSCTRL_reg.FREQ        *= 5;
   cc2420_FSCTRL_reg.FREQ        += 357;
   cc2420_FSCTRL_reg.LOCK_STATUS  = 0;
   cc2420_FSCTRL_reg.LOCK_LENGTH  = 0;
   cc2420_FSCTRL_reg.CAL_RUNNING  = 0;
   cc2420_FSCTRL_reg.CAL_DONE     = 0;
   cc2420_FSCTRL_reg.LOCK_THR     = 1;
   
   cc2420_spiWriteReg(
      CC2420_FSCTRL_ADDR,
      &radio_vars.radioStatusByte,
      *(uint16_t*)&cc2420_FSCTRL_reg
   );
   
   // change state
   radio_vars.state = RADIOSTATE_FREQUENCY_SET;
}

void radio_oscillatorOn(void) {   
   cc2420_spiStrobe(CC2420_SXOSCON, &radio_vars.radioStatusByte);
   while (radio_vars.radioStatusByte.xosc16m_stable==0) {
      cc2420_spiStrobe(CC2420_SNOP, &radio_vars.radioStatusByte);
   }
}

void     radio_oscillatorOff(void){
   cc2420_spiStrobe(CC2420_SXOSCOFF, &radio_vars.radioStatusByte);
   while (radio_vars.radioStatusByte.xosc16m_stable==1) {
      cc2420_spiStrobe(CC2420_SNOP, &radio_vars.radioStatusByte);
   }
}

void radio_rfOff(void) {
   
   // change state
   radio_vars.state = RADIOSTATE_TURNING_OFF;
   
   cc2420_spiStrobe(CC2420_SRFOFF, &radio_vars.radioStatusByte);
   // poipoipoi wait until off
   
   // change state
   radio_vars.state = RADIOSTATE_RFOFF;
}

//===== TX

void radio_loadPacket(uint8_t* packet, uint16_t len) {
   // change state
   radio_vars.state = RADIOSTATE_LOADING_PACKET;
   
   cc2420_spiStrobe(CC2420_SFLUSHTX, &radio_vars.radioStatusByte);
   cc2420_spiWriteFifo(&radio_vars.radioStatusByte, packet, len, CC2420_TXFIFO_ADDR);
   
   // change state
   radio_vars.state = RADIOSTATE_PACKET_LOADED;
}

void radio_txNow(void) {
   // change state
   radio_vars.state = RADIOSTATE_TRANSMITTING;
   
   cc2420_spiStrobe(CC2420_STXON, &radio_vars.radioStatusByte);
}

//===== RX

void radio_rxNow(void) {
  // change state
  radio_vars.state = RADIOSTATE_LISTENING;

  // put radio in reception mode
  cc2420_spiStrobe(CC2420_SRXON, &radio_vars.radioStatusByte);
  cc2420_spiStrobe(CC2420_SFLUSHRX, &radio_vars.radioStatusByte);

  // busy wait until radio really listening
  while (radio_vars.radioStatusByte.rssi_valid==0) {
     cc2420_spiStrobe(CC2420_SNOP, &radio_vars.radioStatusByte);
  }
}

void radio_getReceivedFrame(
      uint8_t* bufRead,
      uint8_t* lenRead,
      uint8_t  maxBufLen,
      int8_t*  rssi,
      uint8_t* lqi,
      uint8_t* crc
   ) {
   
   // read the received packet from the RXFIFO
   cc2420_spiReadRxFifo(&radio_vars.radioStatusByte, bufRead, lenRead, maxBufLen);
   
   // On reception, when MODEMCTRL0.AUTOCRC is set, the CC2420 replaces the
   // received CRC by:
   // - [1B] the rssi, a signed value. The actual value in dBm is that - 45.
   // - [1B] whether CRC checked (bit 7) and LQI (bit 6-0)
   *rssi  =  *(bufRead+*lenRead-2);
   *rssi -= 45;
   *crc   = ((*(bufRead+*lenRead-1))&0x80)>>7;
   *lqi   =  (*(bufRead+*lenRead-1))&0x7f;
}

//=========================== private =========================================

//=========================== callbacks =======================================
