#include "adc_sensor.h"

//=========================== defines =========================================

//=========================== variables =======================================

//=========================== prototype =======================================

void adc_start(void);
void adc_stop(void);

//=========================== public ==========================================

void adc_init(void) {
    
    // stop the sensors
    adc_stop();
    
    // set bits P6.4 and P6.5 as peripherals
    P6SEL |= ((1 << INCH_4) | (1 << INCH_5));
    
    // set ADC config
    ADC12MCTL4 = INCH_4 | SREF_0;
    ADC12MCTL5 = INCH_5 | SREF_0;
    
    adc_start();
}

uint16_t adc_read_light(void) {
    return ADC12MEM5;
}

//=========================== private =========================================

inline void adc_start(void) {
  //setup ADC12 parameters
  //128 sampling cycles
  //reference 2.5V
  //multiple samples
  //reference generator on
  ADC12CTL0 = SHT0_6 | SHT1_6 | REF2_5V | MSC | REFON;
  
  //sampling sourced from the sampling timer
  //repeat-sequence-of-channels
  ADC12CTL1 = SHP | CONSEQ_3;

  //clear all end-of-sequences up to P6.5
  ADC12MCTL0 &= ~EOS;
  ADC12MCTL1 &= ~EOS;
  ADC12MCTL2 &= ~EOS;
  ADC12MCTL3 &= ~EOS;
  ADC12MCTL4 &= ~EOS;
  ADC12MCTL5 |=  EOS;
  
  //set the first conversion register P^.4
  ADC12CTL1 |= CSTARTADD_4;
 
  //enable the conversion
  ADC12CTL0 |= ADC12ON;
  ADC12CTL0 |= ENC;
  
  //start sampling
  ADC12CTL0 |= ADC12SC;
}

inline void adc_stop(void) {
  //stop converting immediately
  ADC12CTL0 &= ~ENC;
  
  //need to remove CONSEQ_3 if not EOS is configured
  ADC12CTL1 &= ~CONSEQ_3;

  //wait for conversion to stop
  while(ADC12CTL1 & ADC12BUSY);

  //clear any pending interrupts
  ADC12IFG = 0;
}