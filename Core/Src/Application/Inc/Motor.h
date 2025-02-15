/* 
 * Author: Syaoran
 * Created on: 2024-11-18
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include "ActiveObject.h"
#include "usart.h"
#include "printf.h"

typedef enum {
    // PWC_TRIGGER_SIG = USER_SIG,
    // COMMAND_SENDED_SIG

    TIMEOUT_100Hz_SIG = USER_SIG,
    COMMAND_SENDED_SIG
} MotorEvent;

struct Motor {
    struct Active super;
    QueueHandle_t pwc_sub;
    QueueHandle_t voltagePublic;
    
    // Methods --------------------------------------------------------
    void (*setVoltage) (char *buffer, float32_t voltage);

    Status (*init) (struct Motor * const self, Event const * const event);
    Status (*wait) (struct Motor * const self, Event const * const event);
    Status (*sending) (struct Motor * const self, Event const * const event);

    /* Hàm này sẽ được gọi để nhận tin nhắn mới từ Matlab */
    void (*public) (QueueHandle_t xQueue, const void * pvItemToQueue);
    void (*publicFromISR) (QueueHandle_t xQueue, const void * pvItemToQueue, BaseType_t *pxHigherPriorityTaskWoken);

};

extern const struct MotorClass {
    void (*new) (struct Motor * const self);
} Motor;

#endif /* __MOTOR_H */