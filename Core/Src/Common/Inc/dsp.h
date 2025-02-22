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
#include "adc.h"

#include "Constants.h"

typedef struct Differentiator Differentiator;

struct Differentiator {
    float32_t preState[2];

    arm_biquad_cascade_df2T_instance_f32 biquad;

    float32_t * filter_state;

    void (*different) (Differentiator * const self, float32_t * input, float32_t * rawDiffOutput);
    void (*filter) (Differentiator * const self, float32_t * rawData, float32_t * filtedData);
};

// ---------------------------------------------------
typedef struct DSP DSP;

struct DSP {
    /* Member-------------------- */
    float32_t rawCurrent;
    float32_t current;
    float32_t currentDiff;

    float32_t motor_angle;
    float32_t raw_motor_velocity;
    float32_t motor_velocity;
    float32_t motorEpsilon;

    float32_t pendulum_angle;
    float32_t raw_pendulum_velocity;
    float32_t pendulum_velocity;
    float32_t pendulumEpsilon;

    StateData motor_state;
    StateData cart_state;
    StateData pendulum_state;
    

    Differentiator * current_differentiator;

    Differentiator * motor_differentiator;
    Differentiator * motor_differentiator_II;

    Differentiator * pendulum_differentiator;

    /* Methods-------------------- */
    void (*procesNewData)(DSP * const self, Sensor const * const encoder_topic, State * const state_topic);
};

/* Hàm dùng để khởi tạo giá trị của struct DSP */
void dsp_new (DSP * const self);

#endif /* __DSP_H */