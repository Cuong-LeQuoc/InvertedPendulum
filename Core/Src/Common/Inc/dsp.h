/* 
 * Author: Syaoran
 * Created on: 2024-11-15
 */
#ifndef __DSP_H
#define __DSP_H

#include "FreeRTOS.h"
#include "arm_math.h"
#include "Topic.h"
#include "Butter_Worth.h"

typedef struct Differentiator Differentiator;

struct Differentiator {
    float32_t Ts;
    float32_t Cen_Diff_Coeffs[3];
    float32_t preState[2];

    arm_biquad_cascade_df2T_instance_f32 filter;

    float32_t * filter_state;

    void (*apply) (Differentiator * const self, float32_t * input, float32_t * output, float32_t * rawOutput);
};

void Differentiator_new(Differentiator * const self, float32_t sample_time, uint8_t num_states, float32_t * filter_coeffs);

// ---------------------------------------------------
typedef struct DSP DSP;

struct DSP {
    /* Member-------------------- */
    float32_t gear_ratio;

    float32_t motor_resolution;
    float32_t pendulum_resolution;

    float32_t motor_angle;
    float32_t pendulum_angle;

    float32_t raw_motor_velocity;
    float32_t raw_pendulum_velocity;

    float32_t motor_velocity;
    float32_t pendulum_velocity;

    StateData motor_state;
    StateData cart_state;
    StateData pendulum_state;

    Differentiator * motor_differentiator;
    Differentiator * pendulum_differentiator;


    /* Methods-------------------- */
    void (*ConvertAngle)(DSP * const self, Encoder const * const encoder_topic);
    void (*filter)(DSP * const self);
    void (*estimate)(DSP * const self);
    void (*procesNewData)(DSP * const self, Encoder const * const encoder_topic, State * const state_topic);
};

/* Hàm dùng để khởi tạo giá trị của struct DSP */
void dsp_new (DSP * const self);

#endif /* __DSP_H */