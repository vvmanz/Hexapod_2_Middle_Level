/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include "std_msgs/msg/int32_multi_array.h"

#include "stdbool.h"
#include "usart.h"
#include "spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t defaultTaskHandle;
osThreadId_t canTaskHandle;
osThreadId_t odometryTaskHandle;
osStaticThreadDef_t rosTaskControlBlock;
uint32_t rosTaskBuffer[8000];

const osThreadAttr_t rosTask_attributes = { .name = "rosTask", .cb_mem =
		&rosTaskControlBlock, .cb_size = sizeof(rosTaskControlBlock),
		.stack_mem = &rosTaskBuffer[0], .stack_size = sizeof(rosTaskBuffer),
		.priority = (osPriority_t) osPriorityAboveNormal, };

const osThreadAttr_t canTask_attributes = { .name = "canTask", .stack_size = 512
		* 4, .priority = (osPriority_t) osPriorityAboveNormal, };

const osThreadAttr_t odometryTask_attributes = { .name = "odometryTask", .stack_size = 512
		* 4, .priority = (osPriority_t) osPriorityAboveNormal2, };

enum {
	KP_ANGLE, KI_ANGLE, KD_ANGLE, KP_SPEED, KI_SPEED, KP_CURRENT, KI_CURRENT
};


/* USER CODE END Variables */
/* Definitions for nulltask */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartNullTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

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
  /* creation of nulltask */

  /* USER CODE BEGIN RTOS_THREADS */
	odometryTaskHandle = osThreadNew(StartOdometryTask, NULL, &odometryTask_attributes);
//	canTaskHandle = osThreadNew(StartCanTask, NULL, &canTask_attributes);
	osThreadNew(StartRosTask, NULL, &rosTask_attributes);

	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartNullTask */
/**
 * @brief  Function implementing the nulltask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartNullTask */
void StartNullTask(void *argument)
{
  /* USER CODE BEGIN StartNullTask */
	/* Infinite loop */
  /* USER CODE END StartNullTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

