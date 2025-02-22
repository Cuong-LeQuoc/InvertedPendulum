#ifndef USART_H
#define USART_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "Computer.h"
#include "Motor.h"

// #define RX2_BUFFER_SIZE 10

void SendBuffer(UART_HandleTypeDef *huart, char* buffer);
// static float32_t StringToFloat(const char *str);

#endif /* USART_H */ 