#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "arm_math.h"

#define SAMPLING_TIME               1e-3f
#define TS                          1e-2f



/**
 * Motor
 */
#define MOTOR_RESOLUTION            2.0f * PI / (4.0f * 500.0f)             /* Encoder: 500 pulse */
#define GEAR_BOX_RATIO              1.0f / 14.0f

/**
 * Cart
 */
#define MODULE                      1.5f
#define TEETH_Z1                    14.0f
#define ANGULAR_TO_DISTANCE         TEETH_Z1 * MODULE * 0.001f / 2.0f       /* m */

/**
 * Pendulum
 */
#define PENDULUM_RESOLUTION         2.0f * PI / (4.0f * 1000.0f)            /* Encoder: 1000 pulse */

/**
 * Sensor: ACS712x20A
 */
#define VCC_ACS712                  2912.0f                                 /* mV */
#define VMID_ACS712                 1470.0f                /* mV */
#define VOFFSET_P_ACS712            1.004f * VCC_ACS712 / 2.0f
#define VOFFSET_N_ACS712            1.01f * VCC_ACS712 / 2.0f
#define SENSITIVITY                 0.103f                                    /* mV/A */
#define ADC_RESOLUTION              4095.0f
/* ADC Configuration */
#define ADC_BUFFER_SIZE             2048                                    /* DMA sẽ tín trung bình của số mẫu này sau mỗi lần đọc */

/**
 * PID Controller
 */
#define P                           10.138f//1.72792f
#define I                           0.1159f //1.466758800000000e-04f
#define D                           0.3f

#endif /* CONSTANTS_H */ 