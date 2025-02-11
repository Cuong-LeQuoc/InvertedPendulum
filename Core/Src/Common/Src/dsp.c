#include "dsp.h"

extern float32_t MotorCoefficients[MOTOR_FILTER_STAGES_NUM * COEFFICIENT_NUMBER];
extern float32_t PendulumCoefficients[MOTOR_FILTER_STAGES_NUM * COEFFICIENT_NUMBER];

static void Differentiator_apply(Differentiator * const self, float32_t * input, float32_t * output, float32_t * rawOutput) {
    float32_t diff_x;
    float32_t filtered_diff_x;
    float32_t x = *input;

    /*Estimate first derivative*/
    diff_x = (3.0f*x - 4.0f*self->preState[0] + 1.0f*self->preState[1]) / (2.0f * self->Ts);

    self->preState[1] = self->preState[0]; self->preState[0] = x;
    
    /*Filter estimated first derivative*/
    self->filter.numStages = 1;
    arm_biquad_cascade_df2T_f32(&self->filter, &diff_x, &filtered_diff_x, 1);

    /*Return output value*/
    *output = filtered_diff_x;
    *rawOutput = diff_x;
}

/**
 * 
 */
void Differentiator_new(Differentiator * const self, float32_t SampleTime, uint8_t num_states, float32_t * filter_coeffs) {
    self->apply = &Differentiator_apply;

    self->Ts = SampleTime;

    self->filter_state = (float32_t *) pvPortMalloc( (2U * (uint32_t) num_states) * sizeof(float32_t) ); 

    arm_biquad_cascade_df2T_init_f32(&self->filter, num_states, filter_coeffs, self->filter_state);	
}


static void ConvertAngle (DSP * const self, Encoder const * const encoder_topic) {
    self->motor_angle       = self->motor_resolution * encoder_topic->MotorCnt;
    self->pendulum_angle    = self->pendulum_resolution * encoder_topic->PendulumCnt;
}

void filter (DSP * const self) {
    self->motor_differentiator->apply(
        self->motor_differentiator,
        &self->motor_angle,
        &self->motor_velocity,
        &self->raw_motor_velocity
    );

    self->pendulum_differentiator->apply(
        self->pendulum_differentiator,
        &self->pendulum_angle,
        &self->pendulum_velocity,
        &self->raw_pendulum_velocity
    );
}

void estimate (DSP * const self) {
    self->motor_state.position  = self->motor_angle;
    self->motor_state.velocity  = self->motor_velocity;
    self->motor_state.rawVel    = self->raw_motor_velocity;

    self->pendulum_state.position   = self->pendulum_angle;
    self->pendulum_state.velocity   = self->pendulum_velocity;
    self->pendulum_state.rawVel     = self->raw_pendulum_velocity;

    self->cart_state.position = self->motor_angle * self->gear_ratio;
    self->cart_state.velocity = self->motor_velocity * self->gear_ratio;
}

void procesNewData (DSP * const self, Encoder const * const encoder_topic, State * const state_topic) {
    self->ConvertAngle(self, encoder_topic);

    self->filter(self);

    self->estimate(self);

    state_topic->Motor      = self->motor_state;
    state_topic->Cart       = self->cart_state;
    state_topic->Pendulum   = self->pendulum_state;
}

void dsp_new (DSP * const self) {
    self->gear_ratio = (1.0f / 14.0f) * ((14.0f * 1.5f * 0.001f) / 2.0f);

    self->motor_resolution = (2.0f * PI) / (4.0f * 500.0f);
    self->pendulum_resolution = (2.0f * PI) / (4.0f * 1000.0f);

    /* Khởi tạo các giá trị ban đầu */
    self->motor_state       = (StateData){0.0f};
    self->cart_state        = (StateData){0.0f};
    self->pendulum_state    = (StateData){0.0f};

    /*Initialize Differentiators*/
    self->motor_differentiator = ( Differentiator * ) pvPortMalloc( sizeof(Differentiator) );
    self->pendulum_differentiator = ( Differentiator * ) pvPortMalloc( sizeof(Differentiator) );
    Differentiator_new(self->motor_differentiator, 1e-3f, MOTOR_FILTER_STAGES_NUM, &MotorCoefficients[0]);
    Differentiator_new(self->pendulum_differentiator, 1e-3f, PENDULUM_FILTER_STAGES_NUM, &PendulumCoefficients[0]);

    /*----------*/
    self->motor_angle = 0.0f;
    self->pendulum_angle = 0.0f; 

    self->raw_motor_velocity = 0.0f; self->motor_velocity = 0.0f;
    self->raw_pendulum_velocity = 0.0f; self->pendulum_velocity = 0.0f;

    /* Assign function pointer */
    self->ConvertAngle  = &ConvertAngle;
    self->filter        = &filter;
    self->estimate      = &estimate;
    self->procesNewData = &procesNewData;
}