/*
 * adc.c
 *
 *  Created on: Apr 5, 2021
 *      Author: gabriel
 */
#include "adc.h"

uint32_t analogRead(ADC_HandleTypeDef *hadc1)
{
  uint32_t ADCValue = 0;
  HAL_ADC_Start(hadc1);
  if (HAL_ADC_PollForConversion(hadc1, 1000000) == HAL_OK) {
      ADCValue = HAL_ADC_GetValue(hadc1);
  }
  HAL_ADC_Stop(hadc1);

  return ADCValue;
}
