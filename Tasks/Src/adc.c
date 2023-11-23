#include "stm32f4xx_hal.h"
#include "main.h"
#include "adc.h"

extern ADC_HandleTypeDef hadc1;

void SelectAdcChannel(uint8_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  switch (channel)
  {
    case 1: sConfig.Channel = ADC_CHANNEL_1; break;
    case 2: sConfig.Channel = ADC_CHANNEL_2; break;
    case 3: sConfig.Channel = ADC_CHANNEL_3; break;
    case 4: sConfig.Channel = ADC_CHANNEL_4; break;
    case 5: sConfig.Channel = ADC_CHANNEL_5; break;
    case 6: sConfig.Channel = ADC_CHANNEL_6; break;
    case 7: sConfig.Channel = ADC_CHANNEL_7; break;
    case 8: sConfig.Channel = ADC_CHANNEL_8; break;
    case 9: sConfig.Channel = ADC_CHANNEL_9; break;
    case 10: sConfig.Channel = ADC_CHANNEL_10; break;
    case 11: sConfig.Channel = ADC_CHANNEL_11; break;
    case 12: sConfig.Channel = ADC_CHANNEL_12; break;
    case 13: sConfig.Channel = ADC_CHANNEL_13; break;
    case 14: sConfig.Channel = ADC_CHANNEL_14; break;
    case 15: sConfig.Channel = ADC_CHANNEL_15; break;
  }
  
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

uint16_t Read_Adc1(uint8_t channel, uint16_t sum_count)
{
	uint32_t sum_value  = 0;
	uint16_t count;
	
	SelectAdcChannel(channel);
	for(count = 0; count < sum_count ; count++)
	{
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1,100);
        sum_value += HAL_ADC_GetValue(&hadc1);     
        HAL_ADC_Stop(&hadc1);
	}
  sum_value = sum_value / sum_count;
	
	return (uint16_t)sum_value;
}

float measure_current(void)
{
    uint16_t adc = 0;
    float current = 0;
    adc = Read_Adc1(5, 100);
    //current = (adc*25)/4095/3;      // diff amp gain = 3, 10/3 = 3.33 multiply by it to get voltage divider gain. (3*3.33 ~= 10)
    current = (adc * 8.3)/4095;         // diff amp gain = 3,  2.5 V = 8.3 A
    return current;
}
