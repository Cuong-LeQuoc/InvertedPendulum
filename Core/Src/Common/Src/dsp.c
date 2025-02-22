#include "dsp.h"

extern float32_t MotorCoefficients[MOTOR_FILTER_STAGES_NUM * COEFFICIENT_NUMBER];
extern float32_t PendulumCoefficients[PENDULUM_FILTER_STAGES_NUM * COEFFICIENT_NUMBER];
extern float32_t CurrentCoefficients[CURRENT_FILTER_STAGES_NUM * COEFFICIENT_NUMBER];

float32_t vol = 0.0f;

static void different(Differentiator * const self, float32_t * input, float32_t * rawDiffOutput) {
    float32_t diff_x;
    float32_t x = *input;

    /*Estimate first derivative*/
    diff_x = (3.0f*x - 4.0f*self->preState[0] + 1.0f*self->preState[1]) / (2.0f * SAMPLING_TIME);

    self->preState[1] = self->preState[0]; self->preState[0] = x;

    /*Return output value*/
    *rawDiffOutput = diff_x;
}

static void filter (Differentiator * const self, float32_t * rawData, float32_t * filtedData) {
    float32_t filtered;
    
    /*Filter estimated first derivative*/
    self->biquad.numStages = 1;
    arm_biquad_cascade_df2T_f32(&self->biquad, rawData, &filtered, 1); // Cần kiểm tra lại con trỏ của rawData

    /*Return output value*/
    *filtedData = filtered;
}


static void Differentiator_new(Differentiator * const self, uint8_t num_states, float32_t * filter_coeffs) {
    self->preState[0] = 0.0f;
    self->preState[1] = 0.0f;
    self->preState[2] = 0.0f;

    self->filter_state = (float32_t *) pvPortMalloc( (2U * (uint32_t) num_states) * sizeof(float32_t) ); 

    arm_biquad_cascade_df2T_init_f32(&self->biquad, num_states, filter_coeffs, self->filter_state);	
}

static void ConvertAngle (DSP * const self, Sensor const * const sensorTopic) {
    self->motor_angle       = MOTOR_RESOLUTION * sensorTopic->motorCnt;
    self->pendulum_angle    = PENDULUM_RESOLUTION * sensorTopic->pendulumCnt;
}

static void ConvertCurrent (DSP * const self, Sensor const * const sensorTopic) {
    float32_t volOut = 0.0f;

    volOut = sensorTopic->currentAdc * VCC_ACS712 / ADC_RESOLUTION;

    vol = volOut;

    self->rawCurrent = (volOut - VMID_ACS712) / SENSITIVITY;    // mA


    
    // if(volOut >= VMID_ACS712) {
    //     self->rawCurrent = (volOut - VOFFSET_P_ACS712) / SENSITIVITY;    // mA
    // }
    // else if(volOut < VMID_ACS712) {
    //     self->rawCurrent = (volOut - VOFFSET_N_ACS712) / SENSITIVITY;    // mA
    // }
}

static void apply (DSP * const self) {
    /* Đạo hàm góc của motor */
    different(
        self->motor_differentiator,
        &self->motor_angle,
        &self->raw_motor_velocity
    );
    /* Lọc vận tốc của motor */
    filter(
        self->motor_differentiator,
        &self->raw_motor_velocity,
        &self->motor_velocity
    );

    /* Đạo hàm vận tốc = gia tốc */
    different(
        self->motor_differentiator_II,
        &self->motor_velocity,
        &self->motorEpsilon
    );

    /* Đạo hàm góc của con lắc */
    different(
        self->pendulum_differentiator,
        &self->pendulum_angle,
        &self->raw_pendulum_velocity
    );

    /* Lọc vận tốc của con lắc */
    filter(
        self->pendulum_differentiator,
        &self->raw_pendulum_velocity,
        &self->pendulum_velocity
    );

    /* Fill dữ liệu dòng điện */
    filter(
        self->current_differentiator,
        &self->rawCurrent,
        &self->current
    );

    /* Đạo hàm cường độ dòng điện */
    different(
        self->current_differentiator,
        &self->current,
        &self->currentDiff
    );

    self->motor_state.position          = self->motor_angle;
    self->motor_state.rawVel            = self->raw_motor_velocity;
    self->motor_state.velocity          = self->motor_velocity; // lọc tín hiệu từ đây
    self->motor_state.acceleration      = self->motorEpsilon;

    self->motor_state.posShaft          = self->motor_angle * GEAR_BOX_RATIO;
    self->motor_state.velShaft          = self->motor_velocity * GEAR_BOX_RATIO;
    

    self->motor_state.rawCurrent        = self->rawCurrent;
    self->motor_state.current           = self->current;

    self->motor_state.currentDiff       = self->currentDiff;

    self->pendulum_state.position       = self->pendulum_angle;
    self->pendulum_state.velocity       = self->pendulum_velocity;
    self->pendulum_state.rawVel         = self->raw_pendulum_velocity;

    self->cart_state.position           = self->motor_angle * GEAR_BOX_RATIO * ANGULAR_TO_DISTANCE;
    self->cart_state.velocity           = self->motor_velocity * GEAR_BOX_RATIO * ANGULAR_TO_DISTANCE;
}

static void procesNewData (DSP * const self, Sensor const * const sensorTopic, State * const state_topic) {
    ConvertAngle(self, sensorTopic);

    ConvertCurrent(self, sensorTopic);

    apply(self);

    state_topic->Motor      = self->motor_state;
    state_topic->Cart       = self->cart_state;
    state_topic->Pendulum   = self->pendulum_state;
}

void dsp_new (DSP * const self) {

    /* Khởi tạo các giá trị ban đầu */
    self->motor_state       = (StateData){0.0f};
    self->cart_state        = (StateData){0.0f};
    self->pendulum_state    = (StateData){0.0f};

    /*Initialize Differentiators*/
    self->current_differentiator        = (Differentiator *) pvPortMalloc(sizeof(Differentiator));  
    self->motor_differentiator          = (Differentiator *) pvPortMalloc(sizeof(Differentiator));
    self->motor_differentiator_II       = (Differentiator *) pvPortMalloc(sizeof(Differentiator));
    self->pendulum_differentiator       = (Differentiator *) pvPortMalloc(sizeof(Differentiator));

    Differentiator_new(self->motor_differentiator, MOTOR_FILTER_STAGES_NUM, &MotorCoefficients[0]);
    Differentiator_new(self->pendulum_differentiator, PENDULUM_FILTER_STAGES_NUM, &PendulumCoefficients[0]);
    Differentiator_new(self->current_differentiator, CURRENT_FILTER_STAGES_NUM, &CurrentCoefficients[0]);

    /*----------*/
    self->motor_angle = 0.0f;
    self->pendulum_angle = 0.0f; 

    self->rawCurrent            = 0.0f; self->current           = 0.0f; self->currentDiff       = 0.0f;
    self->raw_motor_velocity    = 0.0f; self->motor_velocity    = 0.0f; self->motorEpsilon      = 0.0f;
    self->raw_pendulum_velocity = 0.0f; self->pendulum_velocity = 0.0f; self->pendulumEpsilon   = 0.0f;

    /* Assign function pointer */
    self->procesNewData = &procesNewData;
}