/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "definitions.h"
#include "usb_hid_api.h"
#include "app_bluenrg_2.h"
#include "gatt_db.h"
#include "dsp.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint32_t g_adc_buffer[3];
extern uint32_t g_adc2_buffer[3];
TRAP_DRIVE motor;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for commTask */
osThreadId_t commTaskHandle;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void DefaultTask(void *argument);
void LedTask(void *argument);
void ControlTask(void *argument);
void CommTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  init_trap_drive(&motor, &htim1, &htim2, 1);
  init_encoder(&motor.encoder, &htim8);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(DefaultTask, NULL, &defaultTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(LedTask, NULL, &ledTask_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(ControlTask, NULL, &controlTask_attributes);

  /* creation of commTask */
  commTaskHandle = osThreadNew(CommTask, NULL, &commTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_DefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DefaultTask */
void DefaultTask(void *argument)
{
  /* USER CODE BEGIN DefaultTask */
  /* Infinite loop */
  for(;;)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  /* USER CODE END DefaultTask */
}

/* USER CODE BEGIN Header_LedTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedTask */
void LedTask(void *argument)
{
  /* USER CODE BEGIN LedTask */
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));

    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
  /* USER CODE END LedTask */
}

/* USER CODE BEGIN Header_ControlTask */
uint16_t g_encoder = 0;
uint16_t g_dac = 0;
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlTask */
void ControlTask(void *argument)
{
  /* USER CODE BEGIN ControlTask */
  // positive current limit
  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4000);
  // negative current limit
  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
  // start timer to trigger ADC conversions
  HAL_TIM_Base_Start_IT(&htim3);

  enable_trap_drive(&motor, TRUE);

  /* Infinite loop */
  for(;;)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
    update_encoder_position(&motor.encoder);

    update_pwm_cmd(&motor, 1500);
//    update_hall_velocity(&motor, VELOCITY_GAIN);

//    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&g_adc2_buffer, ARRAY_SIZE(g_adc2_buffer));
//
//    HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, g_dac);
//    HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, (4095 - g_dac));
//    g_dac++;
//    g_dac &= 0xFFF;
  }
  /* USER CODE END ControlTask */
}

/* USER CODE BEGIN Header_CommTask */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
int8_t ble_mode = 0;
/* USER CODE END Header_CommTask */
void CommTask(void *argument)
{
  /* USER CODE BEGIN CommTask */
  int32_t milliseconds = 0;
  float sine_scale = 0;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  init_usb_data();

//  int32_t telemetry[2];
//  uint16_t counter = 0;
//  telemetry[0] = 500;
//  MX_BlueNRG_2_Init();
//  ble_set_connectable();

  /* Infinite loop */
  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    sine_scale = sinf(M_PI * (2 * 0.001) * milliseconds ) * 1000;
    if(++milliseconds >= 1000) milliseconds = 0;
    // load telemetry feedback
    set_usb_data32(0, get_usb_data32(0));
    set_usb_data32(1, sine_scale);
    set_usb_data32(2, milliseconds);

    set_usb_mode(get_usb_mode());
    update_usb_timestamp();
    load_usb_tx_data();


//    osDelay(1);
//    MX_BlueNRG_2_Process(xTaskGetTickCount());
//    vTaskDelay(pdMS_TO_TICKS(5));
//
//    ble_mode = get_ble_data8(0);
//
//    // set telemetry data
//    telemetry[0] = get_ble_data16(1);
//    FLOAT_BITS(telemetry[1]) = (1.0F/4095.0F)*counter;
//    set_ble_data(telemetry, 2);
//
//    ++counter;
//    counter &= 0xFFF;
  }
  /* USER CODE END CommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
volatile uint32_t g_call_count = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  g_call_count++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  update_hall_state(&motor.hall);
  update_state_cmd(&motor);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
