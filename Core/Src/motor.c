/*
 * motor.c
 *
 *  Created on: Nov 20, 2019
 *      Author: bpaik
 */

#include "motor.h"
#include "tim.h"
#include "stdlib.h"
#include "limits.h"

//const uint8_t g_hall_table[] = {0, 4, 6, 5, 2, 3, 1, 0};
const uint8_t g_hall_table[] = {0, 4, 2, 3, 6, 5, 1, 0};

void init_trap_drive(TRAP_DRIVE *trap_drive, TIM_HandleTypeDef *pwm_tim, TIM_HandleTypeDef *hall_tim, int32_t direction)
{
  trap_drive->pwm_tim = pwm_tim;
  trap_drive->hall_tim = hall_tim;
  // set default state data
  trap_drive->cmd_state = 1;
  trap_drive->hall_index = (HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin) << 2) |
                           (HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin) << 1) |
                           (HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin));

  trap_drive->hall_state = g_hall_table[trap_drive->hall_index];
  trap_drive->hall_polarity = 1;
  trap_drive->period = 0;

  // initialize duty cycle variables
  trap_drive->pwm_command = 0;
  trap_drive->compare = 0;
  trap_drive->enable = 0;

  // scaling factor for normalized 12-bit duty cycle to compare value
  trap_drive->scale = (float_t)pwm_tim->Init.Period / MAX_DUTY_CYCLE;

  if(direction > 0) {
    trap_drive->polarity = 1;
  }
  else {
    trap_drive->polarity = -1;
  }
}

void enable_trap_drive(TRAP_DRIVE *trap_drive, uint8_t enable)
{
  if(enable != trap_drive->enable) {
    trap_drive->enable = enable;
    update_state_cmd(trap_drive);
  }
}

void update_pwm_cmd(TRAP_DRIVE *trap_drive, int32_t command)
{
 trap_drive->pwm_command = command * trap_drive->polarity;
 trap_drive->compare = abs((int16_t)(trap_drive->scale * trap_drive->pwm_command));

 __HAL_TIM_SET_COMPARE(trap_drive->pwm_tim, TIM_CHANNEL_1, trap_drive->compare);
 __HAL_TIM_SET_COMPARE(trap_drive->pwm_tim, TIM_CHANNEL_2, trap_drive->compare);
 __HAL_TIM_SET_COMPARE(trap_drive->pwm_tim, TIM_CHANNEL_3, trap_drive->compare);
}

void update_state_cmd(TRAP_DRIVE *trap_drive)
{
  if(!trap_drive->enable) {
    disable_trap_drive(trap_drive);
    return;
  }

  if(trap_drive->pwm_command >= 0) {
    trap_drive->cmd_state = trap_drive->hall_state + 1;
    if(trap_drive->cmd_state > 6) {
      trap_drive->cmd_state -= 6;
    }
  }
  else {
    trap_drive->cmd_state = trap_drive->hall_state - 2;
    if(trap_drive->cmd_state < 1) {
      trap_drive->cmd_state += 6;
    }
  }

  // configure PWM registers based on the commanded state
  switch(trap_drive->cmd_state) {

  case(1):
    // disable phase
    SET_PWM3_OFF(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM2_LOW(trap_drive->pwm_tim->Instance);
    // PWM phase
    SET_PWM1_ON(trap_drive->pwm_tim->Instance);
    break;

  case(2):
    // disable phase
    SET_PWM2_OFF(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM3_LOW(trap_drive->pwm_tim->Instance);
    // PWM phase
    SET_PWM1_ON(trap_drive->pwm_tim->Instance);
    break;

  case(3):
    // disable phase
    SET_PWM1_OFF(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM3_LOW(trap_drive->pwm_tim->Instance);
    // PWM phase
    SET_PWM2_ON(trap_drive->pwm_tim->Instance);
    break;

  case(4):
    // disable phase
    SET_PWM3_OFF(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM1_LOW(trap_drive->pwm_tim->Instance);
    // PWM phase
    SET_PWM2_ON(trap_drive->pwm_tim->Instance);
    break;

  case(5):
    // disable phase
    SET_PWM2_OFF(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM1_LOW(trap_drive->pwm_tim->Instance);
    // PWM phase
    SET_PWM3_ON(trap_drive->pwm_tim->Instance);
    break;

  case(6):
    // disable phase
    SET_PWM1_OFF(trap_drive->pwm_tim->Instance);
    // phase lowers
    SET_PWM2_LOW(trap_drive->pwm_tim->Instance);
    // PWM phase
    SET_PWM3_ON(trap_drive->pwm_tim->Instance);
    break;

  default:
    break;
  }
}

void disable_trap_drive(TRAP_DRIVE *trap_drive)
{
  SET_PWM1_OFF(trap_drive->pwm_tim->Instance);
  SET_PWM2_OFF(trap_drive->pwm_tim->Instance);
  SET_PWM3_OFF(trap_drive->pwm_tim->Instance);
}

void update_hall_state(TRAP_DRIVE *trap_drive)
{
  trap_drive->period = __HAL_TIM_GET_COUNTER(trap_drive->hall_tim);
  __HAL_TIM_SET_COUNTER(trap_drive->hall_tim, 0);
  HAL_TIM_Base_Start_IT(trap_drive->hall_tim);

  trap_drive->hall_index = (HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin) << 2) |
                           (HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin) << 1) |
                           (HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin));

  trap_drive->hall_state = g_hall_table[trap_drive->hall_index];
  trap_drive->hall_state_delta = trap_drive->hall_state - trap_drive->hall_state_previous;

  if(trap_drive->hall_state_delta == HALL_ROLLOVER || trap_drive->hall_state_delta == 1) {
    trap_drive->position++;
  }
  else {
    trap_drive->position--;
  }
  trap_drive->hall_state_previous = trap_drive->hall_state;
}

void update_hall_velocity(TRAP_DRIVE *trap_drive, float_t gain)
{
  uint32_t counter = __HAL_TIM_GET_COUNTER(trap_drive->hall_tim);
  trap_drive->position_delta = trap_drive->position - trap_drive->position_previous;

  // valid hall state transition and new velocity measurement
  if(trap_drive->position_delta != 0) {
    trap_drive->hall_polarity = (trap_drive->position_delta > 0) ? 1 : -1;
    if(trap_drive->period > 0) {
      trap_drive->velocity = (gain / trap_drive->period) * trap_drive->hall_polarity;
    }
  }
  // no hall state transition and more time has passed since the previous sample
  else if(counter > trap_drive->period) {
    trap_drive->velocity = (gain / counter) * trap_drive->hall_polarity;
  }
  // timeout
  else if(counter == 0) {
    trap_drive->velocity = 0;
  }
  // store the previous hall state value
  trap_drive->position_previous = trap_drive->position;
}
