#ifndef __DRV_ADC_H
#define __DRV_ADC_H

#include "stm32f10x.h"

typedef struct
{
uint16_t adc_value1;
uint16_t adc_value2;
uint16_t adc_value3;
uint16_t adc_value4;
uint16_t adc_value5;
uint16_t adc_value6;	
}adc_values_t;

#define ADC_MEAN_SIZE         4

#define ADC_SAMPLING_FREQ      100
#define ADC_OVERSAMPLING_FREQ  (ADC_SAMPLING_FREQ * ADC_MEAN_SIZE)

#define ADC_TRIG_PRESCALE       1
#define ADC_TRIG_PRESCALE_FREQ  (72000000 / (ADC_TRIG_PRESCALE + 1))
#define ADC_TRIG_PERIOD         (ADC_TRIG_PRESCALE_FREQ / (ADC_OVERSAMPLING_FREQ))

void ADC_DMA_IRQHandler(void);
void ADC_StartConver(void);
void ADC_StopConver(void);
void adc_hw_init(void);

#endif
