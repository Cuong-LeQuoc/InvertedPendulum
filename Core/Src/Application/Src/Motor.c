#include "Motor.h"

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim6;

struct Active * AO_Motor;
struct Motor * motor;

extern struct Estimator * estimator;

char tx_data[25];

float32_t t = 0.0f;

State setPoint;

static float32_t PID_Controller(State setPoint, State currentValue);

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
            static SignalControl signalControlTopic = {.voltage = 0};
            static State stateTopic = {{0.0f}};
            BaseType_t isSuccessState;
            // is_success = xQueuePeek(self->signalSubsribers, &signalControlTopic, 0);

            // xQueuePeek(self->setPointSubsribers, &setPoint, 0);

            isSuccessState = xQueuePeek(estimator->statePublic, &stateTopic, 0);


            if(isSuccessState) {

                t += 0.01f;
                setPoint.Motor.posShaft = 2.0f *PI*arm_sin_f32(3.0*t);
                setPoint.Motor.velShaft = 2.0* 2.0*PI*arm_cos_f32(3.0*t);

                signalControlTopic.voltage = PID_Controller(setPoint, stateTopic);

                self->setVoltage(tx_data, signalControlTopic.voltage);

                self->public(self->signalSubsribers, &signalControlTopic);

                self->super.handler = (StateHandler) self->sending;
                status = TRAN_STATUS;
            }
            else {
                status = HANDLED_STATUS;
            }

            // if(is_success) {
            //     self->setVoltage(tx_data, signalControlTopic.voltage);

            //     self->super.handler = (StateHandler) self->sending;
            //     status = TRAN_STATUS;
            // }
            // else {
            //     status = HANDLED_STATUS;
            // }

            // sprintf(tx_data, "N1 O d%d\n", pwcDebug);
            // SendBuffer(&huart2, tx_data);

            // self->setVoltage(tx_data, volDebug);
            // self->public(self->voltagePublic, &volDebug);

            // self->super.handler = (StateHandler) self->sending;
            // status = TRAN_STATUS;
        
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

    /* Begin by 0V */
    setVoltage(tx_data, 0);

    /*Cache Ao for using in Encoder driver*/
    AO_Motor = &self->super;
    motor = self;

    // mode_t = (MODE) MAN;

    /*Initialize Queue for Mailbox as subsribers, publishers*/
    self->signalSubsribers      = xQueueCreate(1, sizeof(SignalControl));
    self->setPointSubsribers    = xQueueCreate(1, sizeof(State));
}

const struct MotorClass Motor = { .new = &new };


static float32_t PID_Controller(State setPoint, State currentValue) {
    static float32_t Iterm = 0, pre_x_error;
    float32_t uout = 0;

    float32_t x_error = setPoint.Motor.posShaft - currentValue.Motor.posShaft;
    float32_t v_error = setPoint.Motor.velShaft - currentValue.Motor.velShaft;

    float32_t Pterm = P*x_error;

    Iterm += I * TS * (x_error + pre_x_error)/2.0f;
    
    float32_t Dterm = D*v_error;//x_error/TS;
    pre_x_error = x_error;

    uout = Pterm + Iterm + Dterm;

    // if(uout > 11.0f) uout = 11.0f;
    // else if(uout < -11.0f) uout = -11.0f;

    if(uout > 0.0f) {
        if(uout < 0.9f) uout = 0.9f;
        else if(uout > 11.0f) uout = 11.0f;
    }
    else {
        if(uout > -0.9f) uout = -0.9f;
        else if(uout < -11.0f) uout = -11.0f;
    }
    return uout; 
}

