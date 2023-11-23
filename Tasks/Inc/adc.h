#ifndef __ADC_H
#define __ADC_H

#include "stm32f4xx_hal.h"
#include "main.h"

void SelectAdcChannel(uint8_t channel);
uint16_t Read_Adc1(uint8_t channel, uint16_t sum_count);
float measure_current(void);

#endif /*__ ADC_H */
