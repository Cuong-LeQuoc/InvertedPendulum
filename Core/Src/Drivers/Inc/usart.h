#ifndef USART_H
#define USART_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "Computer.h"
#include "Motor.h"

void SendBuffer(UART_HandleTypeDef *huart, char* buffer);

#endif /* USART_H */ 