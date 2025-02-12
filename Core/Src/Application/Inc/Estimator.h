/* 
 * Author: Syaoran
 * Created on: 2024-11-18
 */
#ifndef __ESTIMATOR_H
#define __ESTIMATOR_H

#include "stm32f4xx_hal.h"
#include "ActiveObject.h"
#include "Topic.h"
#include "dsp.h"
#include "computer.h"

typedef enum {
    /* Sự kiện sẽ được kích hoạt khi timer trigger*/
    TIMEOUT_1kHz_SIG = USER_SIG
} EstimatorEvent;

struct Estimator {
    
    /* Members---------- */
    struct Active super;   /* Kế thừa lớp Active-Object */
    struct DSP * data_processor;
    QueueHandle_t encoderSubsriber;
    QueueHandle_t statePublic;


    /* Methods---------- */
    Status (*init)(struct Estimator * const self, Event const * const event);
    Status (*wait) (struct Estimator * const self, Event const * const event);
    /*------------------*/
    void (*public) (QueueHandle_t xQueue, const void * pvItemToQueue);
    void (*publicFromISR) (QueueHandle_t xQueue, const void * pvItemToQueue,
                           BaseType_t * pxHigherPriorityTaskWoken);

};

extern const struct EstimatorClass {
    void (*new) (struct Estimator * self);
} Estimator;

#endif /* __ESTIMATOR_H */