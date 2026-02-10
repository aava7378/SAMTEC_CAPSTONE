/*
 * thermocouple.c
 *
 *  Created on: Nov 8, 2025
 *      Author: aaron
 */
//#include "thermocouple.h"
//#include "adc.h"
//
//#define ADC_CH 1  // PA1 -> ADC1_IN1
//
//static inline int32_t adc_to_mV(uint16_t adc)
//{
//    return (int32_t)adc * TC_VREF_mV / ((1U<<TC_ADC_BITS) - 1U); // 0..4095
//}
//
//void tc_init(void)
//{
//    adc1_init_single(ADC_CH);
//}
//
//int32_t tc_read_c_x10(void)
//{
//    uint16_t raw = adc1_read_single(ADC_CH);
//    int32_t mv   = adc_to_mV(raw);                 // mV
//    int32_t dmv  = mv - AD8495_OFFSET_mV;          // mV above 0°C
//    // °C = (dmv*1000 uV/mV) / slope; return ×10 for one decimal
//    int32_t cx10 = (dmv * 1000 * 10) / AD8495_SLOPE_uV_C;
//    return cx10;
//}

#include "thermocouple.h"
#include "adc.h"

static inline int32_t adc_to_mV(uint16_t adc)
{
    return (int32_t)adc * TC_VREF_mV / ((1U<<TC_ADC_BITS) - 1U); // 0..4095
}

void tc_init(void)
{
    // We will use ADC1 for multiple channels (PA0 and PA1)
    adc1_init_single(0);               // init/cal ADC1 (channel arg not "fixed" anymore)

    // Ensure BOTH pins are analog and sample time is set for both channels
    adc1_enable_gpio_analog(0);        // PA0
    adc1_enable_gpio_analog(1);        // PA1
    adc1_set_sample_time_max(0);
    adc1_set_sample_time_max(1);
}

int32_t tc_read_c_x10_ch(uint8_t adc1_channel)
{
    uint16_t raw = adc1_read_single(adc1_channel);
    int32_t mv   = adc_to_mV(raw);                 // mV
    int32_t dmv  = mv - AD8495_OFFSET_mV;          // mV above 0°C
    int32_t cx10 = (dmv * 1000 * 10) / AD8495_SLOPE_uV_C; // °C * 10
    return cx10;
}

// Default: TC1 on PA0 / ADC1_IN0
int32_t tc_read_c_x10(void)
{
    return tc_read_c_x10_ch(0);
}



