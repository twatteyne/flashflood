#include "address_setup.h"
#include "opendefs.h"
#include "radio.h"
#include "cc2420.h"

// ========================== variable ========================================

static  uint8_t cc2420_shortadr_cycle_1[2] = {0x11,0x11};
static  uint8_t cc2420_shortadr_cycle_2[2] = {0x22,0x22};
static  uint8_t cc2420_shortadr_cycle_3[2] = {0x33,0x33};
static  uint8_t cc2420_shortadr_cycle_4[2] = {0x44,0x44};
static  uint8_t cc2420_shortadr_cycle_5[2] = {0x55,0x55};

static  uint8_t cc2420_panid_cycle_1[2]    = {0x11,0x11};
static  uint8_t cc2420_panid_cycle_2[2]    = {0x22,0x22};
static  uint8_t cc2420_panid_cycle_3[2]    = {0x33,0x33};
static  uint8_t cc2420_panid_cycle_4[2]    = {0x44,0x44};
static  uint8_t cc2420_panid_cycle_5[2]    = {0x55,0x55};

static  uint8_t cc2420_ieeeadr_cycle_1[8]  = {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11};
static  uint8_t cc2420_ieeeadr_cycle_2[8]  = {0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22};
static  uint8_t cc2420_ieeeadr_cycle_3[8]  = {0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33};
static  uint8_t cc2420_ieeeadr_cycle_4[8]  = {0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44};
static  uint8_t cc2420_ieeeadr_cycle_5[8]  = {0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};

//=========================== prototype =======================================

void address_setup(uint8_t myId, uint8_t* cycleId){
   cc2420_status_t cc2420_status;
  
    // prepare radio
    radio_rfOn();
    switch (myId){
    case SOURCE_ID:
        // write short address
        cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR,
                           &cc2420_status,
                           &cc2420_shortadr_cycle_1[0],
                           2);
        // write panId
        cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,
                           &cc2420_status,
                           &cc2420_panid_cycle_1[0],
                           2);
        // write 64-bit ieee address
        cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,
                           &cc2420_status,
                           &cc2420_ieeeadr_cycle_1[0],
                           8);
        *cycleId   = 0x11;
        break;
    case FIRST_HOP_1:
    case FIRST_HOP_2:
        // write short address
        cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR,
                           &cc2420_status,
                           &cc2420_shortadr_cycle_2[0],
                           2);
        // write panId
        cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,
                           &cc2420_status,
                           &cc2420_panid_cycle_2[0],
                           2);
        // write 64-bit ieee address
        cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,
                           &cc2420_status,
                           &cc2420_ieeeadr_cycle_2[0],
                           8);
        *cycleId   = 0x22;
        break;
    case SECOND_HOP_1:
    case SECOND_HOP_2:
        // write short address
        cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR,
                           &cc2420_status,
                           &cc2420_shortadr_cycle_3[0],
                           2);
        // write panId
        cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,
                           &cc2420_status,
                           &cc2420_panid_cycle_3[0],
                           2);
        // write 64-bit ieee address
        cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,
                           &cc2420_status,
                           &cc2420_ieeeadr_cycle_3[0],
                           8);
        *cycleId   = 0x33;
        break;
    case THIRD_HOP_1:
    case THIRD_HOP_2:
        // write short address
        cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR,
                           &cc2420_status,
                           &cc2420_shortadr_cycle_4[0],
                           2);
        // write panId
        cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,
                           &cc2420_status,
                           &cc2420_panid_cycle_4[0],
                           2);
        // write 64-bit ieee address
        cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,
                           &cc2420_status,
                           &cc2420_ieeeadr_cycle_4[0],
                           8);
        *cycleId   = 0x44;
        break;
    case FOURTH_HOP_1:
    case FOURTH_HOP_2:
        // write short address
        cc2420_spiWriteRam(CC2420_RAM_SHORTADR_ADDR,
                           &cc2420_status,
                           &cc2420_shortadr_cycle_5[0],
                           2);
        // write panId
        cc2420_spiWriteRam(CC2420_RAM_PANID_ADDR,
                           &cc2420_status,
                           &cc2420_panid_cycle_5[0],
                           2);
        // write 64-bit ieee address
        cc2420_spiWriteRam(CC2420_RAM_IEEEADR_ADDR,
                           &cc2420_status,
                           &cc2420_ieeeadr_cycle_5[0],
                           8);
        *cycleId   = 0x55;
        break;
    case DESTINATION_ID:
        // no such mote
        *cycleId   = 0x66;
        break;
    default:
        // nothing to do
        break;
    }
}