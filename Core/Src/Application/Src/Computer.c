#include "Computer.h"

extern UART_HandleTypeDef huart3;

extern struct Estimator * estimator;
extern struct Motor * motor;

struct Active * AO_Computer;
struct Computer * computer;

static char tx_data[50];

static Status init (struct Computer * const self, Event const * const event) {
    Status status = TRAN_STATUS;
    self->super.handler = (StateHandler) self->wait;
    return status;
}

static Status sending (struct Computer * const self, Event const * const event) {
    Status status;

    switch (event->signal) {
        case ENTRY_SIG:
            status = HANDLED_STATUS;
            break;

        case SENSOR_SENDED_SIG:
            self->super.handler = (StateHandler)self->wait;
            status = TRAN_STATUS;
            break;

        case EXIT_SIG:
            status = HANDLED_STATUS;
            break;
        
        default:
            status = IGNORED_STATUS;
            break;
    }

    return status;
}

static Status wait (struct Computer *const self, Event const * const event) {
    Status status;
    
    switch (event->signal) {
        case ENTRY_SIG:  
            status = HANDLED_STATUS;
            break;

        case STATE_UPDATED_SIG:
            static State stateTopic = {{0.0f}};
            float32_t voltage = 0;
            BaseType_t is_success;
            is_success = xQueuePeek(estimator->statePublic, &stateTopic, 0);
            xQueuePeek(motor->voltagePublic, &voltage, 0);

            if(is_success) {
                sprintf(tx_data,
                        "S%0.6f %0.6f %0.6f %0.6f\n",
                        voltage,
                        stateTopic.Motor.position,
                        stateTopic.Motor.velocity,
                        stateTopic.Motor.rawVel
                );
                SendBuffer(&huart3, tx_data);

                self->super.handler = (StateHandler) self->sending;
                status = TRAN_STATUS;
            }
            else {
                status = HANDLED_STATUS;
            }
        
            break;

        case EXIT_SIG:
            status = HANDLED_STATUS;
            break;
        
        default:
            status = IGNORED_STATUS;
            break;
    }
    return status;
}

static void public(QueueHandle_t xQueue, const void * pvItemToQueue) {

    xQueueOverwrite(xQueue, pvItemToQueue);
}

static void publicFromISR(QueueHandle_t xQueue, const void * pvItemToQueue, BaseType_t *pxHigherPriorityTaskWoken) {
  xQueueOverwriteFromISR(xQueue, pvItemToQueue, pxHigherPriorityTaskWoken);
}

static void new (struct Computer * const self) {
    /* Assign methods */
    self->init          = &init;
    self->sending       = &sending;
    self->wait          = &wait;
    self->public        = &public;
    self->publicFromISR = &publicFromISR;

    Active_new(&self->super, (StateHandler) &init);

    AO_Computer = &self->super;
    computer = self;

    /* Initialize Queue for Mailbox as subsribers, publishers */
    // self->state_sub = estimator->state_pub;
    self->received_message_sub = xQueueCreate(1, sizeof(RecivedMessage));
}

const struct ComputerClass Computer = { .new = &new };