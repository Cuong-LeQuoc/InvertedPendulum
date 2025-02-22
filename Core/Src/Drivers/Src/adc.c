#include "adc.h"

extern struct Active * AO_Estimator;
extern struct Estimator * estimator;

uint16_t adcBuffer[ADC_BUFFER_SIZE];

extern Sensor sensorTopic;     /* Đồng bộ data với Sensor của timer */

/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(hadc->Instance == ADC1) {
        uint64_t total = 0;

        for(uint32_t i=0; i < ADC_BUFFER_SIZE; i++) {
            total += adcBuffer[i];
        }
        sensorTopic.currentAdc = total / (1.0f * ADC_BUFFER_SIZE);

        estimator->publicFromISR(estimator->sensorSubsribers, &sensorTopic, &xHigherPriorityTaskWoken);
    }

    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}