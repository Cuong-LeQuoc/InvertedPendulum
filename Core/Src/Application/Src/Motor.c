#include "Motor.h"

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim6;

struct Active * AO_Motor;
struct Motor * motor;
char tx_data[25];
int32_t pwcDebug;
float32_t volDebug;

static Status init (struct Motor * const self, Event const * const event) {
    Status status = TRAN_STATUS;
    self->super.handler = (StateHandler) self->wait;
    return status;
}

static Status sending (struct Motor * const self, Event const * const event) {
    Status status;
    
    switch (event->signal) {
        case ENTRY_SIG:
            status = HANDLED_STATUS;
            break;

        case COMMAND_SENDED_SIG:
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

static Status wait (struct Motor * const self, Event const * const event) {
    Status status;
    
    switch (event->signal) {
        case ENTRY_SIG:
            HAL_TIM_Base_Start_IT(&htim6);
            status = HANDLED_STATUS;
            break;

        case TIMEOUT_100Hz_SIG:
            // static PWC pwcTopic = {0};
            // BaseType_t is_success;
            // is_success = xQueuePeek(self->pwc_sub, &pwcTopic, 0);

            // if(is_success) {
            //     sprintf(tx_data, "N1 O d%d\n", pwcTopic.d);
            //     SendBuffer(&huart2, tx_data);

            //     self->super.handler = (StateHandler) self->sending;
            //     status = TRAN_STATUS;
            // }
            // else {
            //     status = HANDLED_STATUS;
            // }

            // sprintf(tx_data, "N1 O d%d\n", pwcDebug);
            // SendBuffer(&huart2, tx_data);

            self->setVoltage(tx_data, volDebug);
            self->public(self->voltagePublic, &volDebug);

            self->super.handler = (StateHandler) self->sending;
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

static void public (QueueHandle_t xQueue, const void * pvItemToQueue) {
    xQueueOverwrite(xQueue, pvItemToQueue);
}

static void publicFromISR (QueueHandle_t xQueue, const void * pvItemToQueue,
                           BaseType_t *pxHigherPriorityTaskWoken)
{
    xQueueOverwriteFromISR(xQueue, pvItemToQueue, pxHigherPriorityTaskWoken);
}

static void setVoltage (char *buffer, float32_t voltage) {
    int16_t pwc = 0;

    if(voltage > 0) {
        pwc = (int16_t) (83.296855952222570f * voltage + 6.699955398806081f);
    } else {
        pwc = (int16_t) (84.226798317384890f * voltage - 21.820956466774298f);
    }

    sprintf(buffer, "N1 O d%d\n", (int)pwc);
    SendBuffer(&huart2, buffer);
}

static void new(struct Motor * const self) {
    /* Assign Methods */
    self->setVoltage    = &setVoltage;
    self->init          = &init;
    self->wait          = &wait;
    self->sending       = &sending;
    self->public        = &public;
    self->publicFromISR = &publicFromISR;

    /*Initialize members*/
    Active_new(&self->super, (StateHandler)&init);

    /*Cache Ao for using in Encoder driver*/
    AO_Motor = &self->super;
    motor = self;

    /*Initialize Queue for Mailbox as subsribers, publishers*/
    self->pwc_sub       = xQueueCreate( 1, sizeof( PWC ) );
    self->voltagePublic = xQueueCreate(1, sizeof(float32_t));
}

const struct MotorClass Motor = { .new = &new };