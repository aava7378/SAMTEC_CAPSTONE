/*
 * thermocouple.c
 *
 *  Created on: Nov 8, 2025
 *      Author: aaron
 */
#include "thermocouple.h"
#include "adc.h"

#define ADC_CH 1  // PA1 -> ADC1_IN1

static inline int32_t adc_to_mV(uint16_t adc)
{
    return (int32_t)adc * TC_VREF_mV / ((1U<<TC_ADC_BITS) - 1U); // 0..4095
}

void tc_init(void)
{
    adc1_init_single(ADC_CH);
}

int32_t tc_read_c_x10(void)
{
    uint16_t raw = adc1_read_single(ADC_CH);
    int32_t mv   = adc_to_mV(raw);                 // mV
    int32_t dmv  = mv - AD8495_OFFSET_mV;          // mV above 0°C
    // °C = (dmv*1000 uV/mV) / slope; return ×10 for one decimal
    int32_t cx10 = (dmv * 1000 * 10) / AD8495_SLOPE_uV_C;
    return cx10;
}


