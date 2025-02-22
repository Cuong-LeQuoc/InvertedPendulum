#include "usart.h"

extern struct Active * AO_Computer;
extern struct Active * AO_Motor;

extern struct Motor * motor;

extern uint8_t rxByte;

RecivedMessage recivedMsg = {.maxLength = 10, .index = 0, .dataComming = 0};

static float32_t ExtractMessage(const char *str) {
  float32_t floatNumber = 0.0f, fraction = 0.1f, sign = 1.0f;
  int checkFraction = 0;

  if (*str == '-') { // Kiểm tra dấu âm
    sign *= -1.0f;
    str++;
  }

  while (*str) {
    if (*str == '.') {  // Nếu gặp dấu '.'
      checkFraction = 1;
    }
    else if (*str >= '0' && *str <= '9') {  // Chỉ xử lý số
      float32_t eachNumber = (float32_t)(*str - '0');
      if (checkFraction) { // Xử lý phần thập phân
        floatNumber += eachNumber * fraction;
        fraction *= 0.1;
      } else {
        floatNumber = floatNumber * 10.0f + eachNumber;  // Xử lý phần nguyên
      }
    }
    str++;
  }

  return sign*floatNumber;
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if(huart->Instance == USART3) {
    

    // static RecivedMessage recivedMsg = {.maxLength = 10, .index = 0, .dataComming = 0}; 
  
    if(recivedMsg.dataComming == 1) { /* Have recived 'V' */
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

      if(rxByte != 'V' && rxByte != 10) { /* Not 'V' and '\n' */
        recivedMsg.message[recivedMsg.index] = rxByte;
        recivedMsg.index++;

        if(recivedMsg.index > recivedMsg.maxLength) {
          recivedMsg.index = 0;
          recivedMsg.dataComming = 0;
        }
      }

      else if(rxByte == 10)  { /* End of Line '\n' */
        // static SignalControl signalTopic = {.voltage = 0};
        static State stateTopic = {{0.0f}};
        // static const Event pwc_evt = {VOLTAGE_TRIGGER_SIG};
        recivedMsg.message[recivedMsg.index] = '\0';

        // signalTopic.voltage = ExtractMessage(recivedMsg.message);
        stateTopic.Motor.posShaft = ExtractMessage(recivedMsg.message);
        stateTopic.Motor.velShaft = 0;
        xQueueOverwriteFromISR(motor->setPointSubsribers, &stateTopic, &xHigherPriorityTaskWoken);
        // xQueueOverwriteFromISR(motor->signalSubsribers, &signalTopic, &xHigherPriorityTaskWoken);
        // AO_Motor->postFromISR(AO_Motor, &pwc_evt, &xHigherPriorityTaskWoken);

        memset(recivedMsg.message, 0, recivedMsg.index);
        recivedMsg.index = 0;
        recivedMsg.dataComming = 0;
      }
    }
    else {
      if(rxByte == 'V') { /* Witnessed double 'V' */
        if(recivedMsg.index != 0) recivedMsg.index = 0;
        recivedMsg.dataComming = 1;
      }          
    }

    /*Enable Rx Interrupt, waiting for new byte data*/
    HAL_UART_Receive_IT(huart, &rxByte, 1);
  }

  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

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

