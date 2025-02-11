#include "timer.h"

extern struct Active * AO_Estimator;

/**
  * @brief  Input Capture callback in non-blocking mode
  * @param  htim TIM IC handle
  * @retval None
  */
 void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM2) {
  
    }
  
    else if(htim->Instance == TIM3) {
      
    }
  }
  
/**
* @brief  Period elapsed callback in non-blocking mode
* @param  htim TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // if(htim->Instance == TIM6) {

  // }

  if(htim->Instance == TIM7) {
    static const Event time_evt = {.signal = TIMEOUT_1kHz_SIG};
    AO_Estimator->postFromISR(AO_Estimator, &time_evt, &xHigherPriorityTaskWoken);
  }

  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}