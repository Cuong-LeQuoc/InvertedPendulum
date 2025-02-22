#include "timer.h"

extern struct Active * AO_Estimator;
extern struct Estimator * estimator;

extern struct Active * AO_Motor;
extern struct Motor * motor;

Sensor sensorTopic;

/**
  * @brief  Input Capture callback in non-blocking mode
  * @param  htim TIM IC handle
  * @retval None
  */
 void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if(htim->Instance == TIM2) {
    sensorTopic.motorCnt = (int32_t)((uint32_t)__HAL_TIM_GET_COUNTER(htim));
    estimator->publicFromISR(estimator->sensorSubsribers, &sensorTopic, &xHigherPriorityTaskWoken);
  }
  else if(htim->Instance == TIM3) { // Đọc encoder của pendulum
    sensorTopic.motorCnt = (int16_t)((uint32_t)__HAL_TIM_GET_COUNTER(htim));
    estimator->publicFromISR(estimator->sensorSubsribers, &sensorTopic, &xHigherPriorityTaskWoken);
  }

  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
  
/**
* @brief  Period elapsed callback in non-blocking mode
* @param  htim TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if(htim->Instance == TIM6) {
    static const Event time_evt = {.signal = TIMEOUT_100Hz_SIG};
    AO_Motor->postFromISR(AO_Motor, &time_evt, &xHigherPriorityTaskWoken);
  }

  if(htim->Instance == TIM7) {
    static const Event time_evt = {.signal = TIMEOUT_1kHz_SIG};
    AO_Estimator->postFromISR(AO_Estimator, &time_evt, &xHigherPriorityTaskWoken);
  }

  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}