#include "usart.h"

extern struct Active * AO_Computer;
extern struct Active * AO_Motor;

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if(huart->Instance == USART2) {
    static const Event motor_sended_event = {.signal = COMMAND_SENDED_SIG};
    AO_Motor->postFromISR(AO_Motor, &motor_sended_event, &xHigherPriorityTaskWoken);
  } else if(huart->Instance == USART3) {
    static const Event sensor_event = {.signal = SENSOR_SENDED_SIG};
    AO_Computer->postFromISR(AO_Computer, &sensor_event, &xHigherPriorityTaskWoken);
  }
  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void SendBuffer(UART_HandleTypeDef *huart, char* buffer) {
    uint64_t length = strlen(buffer);
    HAL_UART_Transmit_IT(huart, (uint8_t*) buffer, length);
}
