#ifndef TOPIC_H
#define TOPIC_H

#include "arm_math.h"

typedef struct {
  int16_t pendulumCnt;
  int32_t motorCnt;

  float32_t currentAdc;
} Sensor;

typedef struct {
  /* Raw Data */
  float32_t rawPos;
  float32_t rawVel;

  /* Observation*/
  float32_t position;
  float32_t velocity;
  float32_t acceleration;

  /* Motor */
  float32_t posShaft;
  float32_t velShaft;

  float32_t rawCurrent;
  float32_t current;

  float32_t currentDiff;
} StateData;

typedef struct {
  StateData Motor;
  StateData Cart;
  StateData Pendulum;
} State;

/* Pulse width command */
typedef struct {
  int16_t d;
  float32_t voltage;
  /* Desired (Setpoint) */
  float32_t xd;
  float32_t vd;
  float32_t ad;
} SignalControl;

typedef struct {
  char message[10];
  int8_t maxLength;
  uint8_t index;
  uint8_t dataComming;

} RecivedMessage;

#endif /* TOPIC_H */ 