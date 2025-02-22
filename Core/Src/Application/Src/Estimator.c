#include "Estimator.h"

extern uint16_t adcBuffer[ADC_BUFFER_SIZE];

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim7;

extern struct Computer * computer;
struct Active * AO_Estimator;
struct Estimator * estimator;


Status init (struct Estimator * const self, Event const * const event) {
    Status status = TRAN_STATUS;

    self->super.handler = (StateHandler) self->wait;

    return status;
}

Status wait (struct Estimator * const self, Event const * const event) {
    Status status;

    switch (event->signal) {
        case ENTRY_SIG:
            HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, ADC_BUFFER_SIZE);
            HAL_TIM_Base_Start_IT(&htim7);
            // HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
            HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);

            status = HANDLED_STATUS;
            break;

        case TIMEOUT_1kHz_SIG:
            static Sensor sensorTopic = {0.0f};
            static State stateTopic = {{0.0f}};

            BaseType_t isSuccess = xQueuePeek(self->sensorSubsribers, &sensorTopic, 0);
            

            if(isSuccess) {
                self->dataProcessor->procesNewData(self->dataProcessor, &sensorTopic, &stateTopic);
            }

            self->public(self->statePublic, &stateTopic);

            static const Event stateEvent = {.signal = STATE_UPDATED_SIG};
            AO_Estimator->post(&computer->super, &stateEvent);

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
                           BaseType_t * pxHigherPriorityTaskWoken)
{
    xQueueOverwriteFromISR(xQueue, pvItemToQueue, pxHigherPriorityTaskWoken);
}

static void new (struct Estimator * const self) {

    /* Assign methods*/
    self->init          = &init;
    self->wait          = &wait;
    self->public        = &public;
    self->publicFromISR = &publicFromISR;

    /* Khởi tạo một Active Object mới kế thừa từ class Avtive Object */
    Active_new(&self->super, (StateHandler) &init);

    /* Cấp phát động cho biến kiểu DSP để lọc và xử lý data */
    self->dataProcessor = (struct DSP *) pvPortMalloc(sizeof(struct DSP));
    dsp_new(self->dataProcessor);

    AO_Estimator = &self->super;
    estimator = self;

    self->sensorSubsribers      = xQueueCreate(1, sizeof(Sensor));
    self->statePublic           = xQueueCreate(1, sizeof(State));
}

const struct EstimatorClass Estimator = { .new = &new };