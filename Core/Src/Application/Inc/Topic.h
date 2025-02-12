#ifndef TOPIC_H
#define TOPIC_H

#include "arm_math.h"

typedef struct {
  int16_t pendulumCnt;
  int32_t motorCnt;
} Encoder;

typedef struct {
  /* Raw Data */
  float32_t rawPos;
  float32_t rawVel;
  float32_t rawAcc;

  /* Observation*/
  float32_t position;
  float32_t velocity;
  float32_t acceleration;
} StateData;

typedef struct {
  StateData Motor;
  StateData Cart;
  StateData Pendulum;
} State;

/* Pulse width command */
typedef struct {
  int16_t d;
  /* Desired (Setpoint) */
  float32_t xd;
  float32_t vd;
  float32_t ad;
} PWC;

typedef struct {
  char message[7];
  uint8_t length;
} RecivedMessage;

#endif /* TOPIC_H */ 