/*
 * state_space.h
 *
 *  Created on: Oct 28, 2021
 *      Author: BPaik
 */

#ifndef INC_STATE_SPACE_H_
#define INC_STATE_SPACE_H_

#define NUM_STATES      3
#define STATE_OUTPUT    0

typedef struct {
  float A[NUM_STATES][NUM_STATES];
  float B[NUM_STATES];

  float x[NUM_STATES];
  float x_previous[NUM_STATES];
  float x_limit[NUM_STATES];
} STATE_SPACE;

typedef struct {
 STATE_SPACE ss;
 float L[NUM_STATES];
 float error;
 float correction[NUM_STATES];
} STATE_OBSERVER;

typedef struct {
  STATE_OBSERVER obs;
  float K[NUM_STATES];
  float r;
  float u;
  float tracking_gain;
} STATE_CONTROLLER;

void init_state_space(STATE_SPACE *state_space);
void update_states(STATE_SPACE *state_space, float u);
void init_observer(STATE_OBSERVER *observer);
void update_observer(STATE_OBSERVER *observer, float feedback, float u);
void update_control_effort(STATE_CONTROLLER *controller, float command);

#endif /* INC_STATE_SPACE_H_ */
