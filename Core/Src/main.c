/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SaveMotorToSlot(uint8_t motor_idx, uint8_t slot);
void LoadMotorFromSlot(uint8_t motor_idx, uint8_t slot);
void LoadAllCurrentSlots(void);
void SaveAllCurrentSlots(void);
void InitSlotsFromTemplate(void);
static MotorSlot_t slots[NUM_SLOTS][NUM_MOTORS];
static bool slot_initialized[NUM_SLOTS][NUM_MOTORS];

static const MotorSlot_t slot_template[NUM_SLOTS][NUM_MOTORS] =
		{
// slot 0 (Угол скорость ток)
				{
				/* motor 0 (1_1) */{ .kp_angle = 4000, .ki_angle = 4000,
						.kd_angle = 10, .kp_speed = 5, .ki_speed = 5, .kp_curr =
								50, .ki_curr = 3000, .obstacle = 0,
						.speed_limit = 127, .phase = 0, .set_zero = 0 },
				/* motor 1 (1_2) */{ .kp_angle = 4000, .ki_angle = 4000,
						.kd_angle = 10, .kp_speed = 5, .ki_speed = 5, .kp_curr =
								50, .ki_curr = 3000, .obstacle = 0,
						.speed_limit = 127, .phase = 0, .set_zero = 0 },
				/* motor 2 (2_1) */{ .kp_angle = 4000, .ki_angle = 4000,
						.kd_angle = 10, .kp_speed = 5, .ki_speed = 5, .kp_curr =
								50, .ki_curr = 3000, .obstacle = 0,
						.speed_limit = 127, .phase = 0, .set_zero = 0 },
				/* motor 3 (2_2) */{ .kp_angle = 4000, .ki_angle = 4000,
						.kd_angle = 10, .kp_speed = 5, .ki_speed = 5, .kp_curr =
								50, .ki_curr = 3000, .obstacle = 0,
						.speed_limit = 127, .phase = 0, .set_zero = 0 },
				/* motor 4 (3_1) */{ .kp_angle = 4000, .ki_angle = 4000,
						.kd_angle = 10, .kp_speed = 5, .ki_speed = 5, .kp_curr =
								50, .ki_curr = 3000, .obstacle = 0,
						.speed_limit = 127, .phase = 0, .set_zero = 0 },
				/* motor 5 (3_2) */{ .kp_angle = 4000, .ki_angle = 4000,
						.kd_angle = 10, .kp_speed = 5, .ki_speed = 5, .kp_curr =
								50, .ki_curr = 3000, .obstacle = 0,
						.speed_limit = 127, .phase = 0, .set_zero = 0 } },

				// slot 1 ((Угол скорость )
				{
				/* motor 0 (1_1) */{ .kp_angle = 1000, .ki_angle = 2000,
						.kd_angle = 1, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 1 (1_2) */{ .kp_angle = 1000, .ki_angle = 2000,
						.kd_angle = 1, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 2 (2_1) */{ .kp_angle = 1000, .ki_angle = 2000,
						.kd_angle = 1, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 3 (2_2) */{ .kp_angle = 1000, .ki_angle = 2000,
						.kd_angle = 1, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 4 (3_1) */{ .kp_angle = 1000, .ki_angle = 2000,
						.kd_angle = 1, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 5 (3_2) */{ .kp_angle = 1000, .ki_angle = 2000,
						.kd_angle = 1, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 } },
				// slot 2 ((Угол ток )
				{
				/* motor 0 (1_1) */{ .kp_angle = 100, .ki_angle = 400,
						.kd_angle = 1, .kp_speed = 0, .ki_speed = 0, .kp_curr =
								5, .ki_curr = 300, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 1 (1_2) */{ .kp_angle = 100, .ki_angle = 400,
						.kd_angle = 1, .kp_speed = 0, .ki_speed = 0, .kp_curr =
								5, .ki_curr = 300, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 2 (2_1) */{ .kp_angle = 100, .ki_angle = 400,
						.kd_angle = 1, .kp_speed = 0, .ki_speed = 0, .kp_curr =
								5, .ki_curr = 300, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 3 (2_2) */{ .kp_angle = 100, .ki_angle = 400,
						.kd_angle = 1, .kp_speed = 0, .ki_speed = 0, .kp_curr =
								5, .ki_curr = 300, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 4 (3_1) */{ .kp_angle = 100, .ki_angle = 400,
						.kd_angle = 1, .kp_speed = 0, .ki_speed = 0, .kp_curr =
								5, .ki_curr = 300, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 5 (3_2) */{ .kp_angle = 100, .ki_angle = 400,
						.kd_angle = 1, .kp_speed = 0, .ki_speed = 0, .kp_curr =
								5, .ki_curr = 300, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 } },
				// slot 3 ((Угол)
				{
				/* motor 0 (1_1) */{ .kp_angle = 400, .ki_angle = 4000,
						.kd_angle = 40, .kp_speed = 0, .ki_speed = 0, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 1 (1_2) */{ .kp_angle = 400, .ki_angle = 4000,
						.kd_angle = 40, .kp_speed = 0, .ki_speed = 0, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 2 (2_1) */{ .kp_angle = 400, .ki_angle = 4000,
						.kd_angle = 40, .kp_speed = 0, .ki_speed = 0, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 3 (2_2) */{ .kp_angle = 400, .ki_angle = 4000,
						.kd_angle = 40, .kp_speed = 0, .ki_speed = 0, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 4 (3_1) */{ .kp_angle = 400, .ki_angle = 4000,
						.kd_angle = 40, .kp_speed = 0, .ki_speed = 0, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 5 (3_2) */{ .kp_angle = 400, .ki_angle = 4000,
						.kd_angle = 40, .kp_speed = 0, .ki_speed = 0, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 } },
				// slot 4 ((Скорость ток)
				{
				/* motor 0 (1_1) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								5, .ki_curr = 300, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 1 (1_2) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								5, .ki_curr = 300, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 2 (2_1) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								5, .ki_curr = 300, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 3 (2_2) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								5, .ki_curr = 300, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 4 (3_1) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								5, .ki_curr = 300, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 5 (3_2) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								5, .ki_curr = 300, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 } },
				// slot 5 (Скорость)
				{
				/* motor 0 (1_1) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 1 (1_2) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 2 (2_1) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 3 (2_2) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 4 (3_1) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 },
				/* motor 5 (3_2) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								0, .ki_curr = 0, .obstacle = 0, .speed_limit =
								127, .phase = 0, .set_zero = 0 } },
				// slot 6 (Ток)
				{/* motor 0 (1_1) */{ .kp_angle = 0, .ki_angle = 0, .kd_angle =
						0, .kp_speed = 1, .ki_speed = 1, .kp_curr = 5,
						.ki_curr = 1000, .obstacle = 0, .speed_limit = 127,
						.phase = 0, .set_zero = 0 },
				/* motor 1 (1_2) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								5, .ki_curr = 1000, .obstacle = 0,
						.speed_limit = 127, .phase = 0, .set_zero = 0 },
				/* motor 2 (2_1) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								5, .ki_curr = 1000, .obstacle = 0,
						.speed_limit = 127, .phase = 0, .set_zero = 0 },
				/* motor 3 (2_2) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								5, .ki_curr = 1000, .obstacle = 0,
						.speed_limit = 127, .phase = 0, .set_zero = 0 },
				/* motor 4 (3_1) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								5, .ki_curr = 1000, .obstacle = 0,
						.speed_limit = 127, .phase = 0, .set_zero = 0 },
				/* motor 5 (3_2) */{ .kp_angle = 0, .ki_angle = 0,
						.kd_angle = 0, .kp_speed = 1, .ki_speed = 1, .kp_curr =
								5, .ki_curr = 1000, .obstacle = 0,
						.speed_limit = 127, .phase = 0, .set_zero = 0 } } };

// not to transmit
CAN_TxHeaderTypeDef *const TxHdrs1[Q_LEN] = { &TxAngles1, &TxCoeffs1_1,
		&TxCoeffs1_2, &TxCoeffs1_3, &TxCoeffs1_4, &TxCoeffs1_5, &TxCoeffs1_6,
		&TxCoeffs1_7, &TxOther1 };
uint8_t *const TxDats1[Q_LEN] = { TxDataAngles1, TxDataCoeffs1_1,
		TxDataCoeffs1_2, TxDataCoeffs1_3, TxDataCoeffs1_4, TxDataCoeffs1_5,
		TxDataCoeffs1_6, TxDataCoeffs1_7, TxDataOther1 };

CAN_TxHeaderTypeDef *const TxHdrs2[Q_LEN] = { &TxAngles2, &TxCoeffs2_1,
		&TxCoeffs2_2, &TxCoeffs2_3, &TxCoeffs2_4, &TxCoeffs2_5, &TxCoeffs2_6,
		&TxCoeffs2_7, &TxOther2 };
uint8_t *const TxDats2[Q_LEN] = { TxDataAngles2, TxDataCoeffs2_1,
		TxDataCoeffs2_2, TxDataCoeffs2_3, TxDataCoeffs2_4, TxDataCoeffs2_5,
		TxDataCoeffs2_6, TxDataCoeffs2_7, TxDataOther2 };

CAN_TxHeaderTypeDef *const TxHdrs3[Q_LEN] = { &TxAngles3, &TxCoeffs3_1,
		&TxCoeffs3_2, &TxCoeffs3_3, &TxCoeffs3_4, &TxCoeffs3_5, &TxCoeffs3_6,
		&TxCoeffs3_7, &TxOther3 };

uint8_t *const TxDats3[Q_LEN] = { TxDataAngles3, TxDataCoeffs3_1,
		TxDataCoeffs3_2, TxDataCoeffs3_3, TxDataCoeffs3_4, TxDataCoeffs3_5,
		TxDataCoeffs3_6, TxDataCoeffs3_7, TxDataOther3 };

static inline void ProcessQueue(CAN_HandleTypeDef *hcan, uint8_t *queue,
		CAN_TxHeaderTypeDef *const hdrs[], uint8_t *const dats[], size_t len,
		uint32_t *mailbox) {
	Prepare_set_angles();
	Prepare_set_koeffs();
	if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
		HAL_CAN_AddTxMessage(hcan, hdrs[*queue], dats[*queue], mailbox);
		*queue = (*queue + 1) % len;
	}
}
////////////////////////////////////////////////////////////////////////
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_CRC_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	W5500_Init();
	reset_all_variables_to_zero();
	InitSlotsFromTemplate();
	Prepare_set_angles();
	Prepare_set_koeffs();
//	prepare_frames();
//	HAL_CAN_Start(&hcan1);
//	HAL_CAN_Start(&hcan2);
//	HAL_CAN_ActivateNotification(&hcan1,
//			CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR | CAN_IT_RX_FIFO0_MSG_PENDING
//					| CAN_IT_RX_FIFO1_MSG_PENDING);
//	HAL_CAN_ActivateNotification(&hcan2,
//	CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR | CAN_IT_RX_FIFO0_MSG_PENDING);
//
//	HAL_CAN_AddTxMessage(&hcan1, &TxAngles1, TxDataAngles1, &mailbox);
//	HAL_CAN_AddTxMessage(&hcan2, &TxAngles3, TxDataAngles3, &mailbox1);

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* Call init function for freertos objects (in cmsis_os2.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void StartCanTask(void *argument) {
	LoadAllCurrentSlots();

	uint8_t prev[NUM_MOTORS];

	prev[0] = (uint8_t) system_type1_1;
	prev[1] = (uint8_t) system_type1_2;
	prev[2] = (uint8_t) system_type2_1;
	prev[3] = (uint8_t) system_type2_2;
	prev[4] = (uint8_t) system_type3_1;
	prev[5] = (uint8_t) system_type3_2;

	for (;;) {
		if (ros_new_coeffs_flag) {
			SaveAllCurrentSlots();
			ros_new_coeffs_flag = 0;
		}
		uint8_t cur0 = (uint8_t) system_type1_1;
		if (cur0 != prev[0]) {
			if (prev[0] < NUM_SLOTS)
				SaveMotorToSlot(0, prev[0]);
			if (cur0 < NUM_SLOTS)
				LoadMotorFromSlot(0, cur0);
			prev[0] = cur0;
		}

		uint8_t cur1 = (uint8_t) system_type1_2;
		if (cur1 != prev[1]) {
			if (prev[1] < NUM_SLOTS)
				SaveMotorToSlot(1, prev[1]);
			if (cur1 < NUM_SLOTS)
				LoadMotorFromSlot(1, cur1);
			prev[1] = cur1;
		}

		uint8_t cur2 = (uint8_t) system_type2_1;
		if (cur2 != prev[2]) {
			if (prev[2] < NUM_SLOTS)
				SaveMotorToSlot(2, prev[2]);
			if (cur2 < NUM_SLOTS)
				LoadMotorFromSlot(2, cur2);
			prev[2] = cur2;
		}

		uint8_t cur3 = (uint8_t) system_type2_2;
		if (cur3 != prev[3]) {
			if (prev[3] < NUM_SLOTS)
				SaveMotorToSlot(3, prev[3]);
			if (cur3 < NUM_SLOTS)
				LoadMotorFromSlot(3, cur3);
			prev[3] = cur3;
		}

		uint8_t cur4 = (uint8_t) system_type3_1;
		if (cur4 != prev[4]) {
			if (prev[4] < NUM_SLOTS)
				SaveMotorToSlot(4, prev[4]);
			if (cur4 < NUM_SLOTS)
				LoadMotorFromSlot(4, cur4);
			prev[4] = cur4;
		}

		uint8_t cur5 = (uint8_t) system_type3_2;
		if (cur5 != prev[5]) {
			if (prev[5] < NUM_SLOTS)
				SaveMotorToSlot(5, prev[5]);
			if (cur5 < NUM_SLOTS)
				LoadMotorFromSlot(5, cur5);
			prev[5] = cur5;
		}

		if (flag_of_tx == 1 && flag_of_rx == 2) {
			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
				flag_of_tx = flag_of_rx = 0;
				ProcessQueue(&hcan1, &queue1, TxHdrs1, TxDats1, Q_LEN,
						&mailbox);
			}
		} else if (flag_of_tx == 1 && flag_of_rx == 1) {
			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
				flag_of_tx = flag_of_rx = 0;
				ProcessQueue(&hcan1, &queue2, TxHdrs2, TxDats2, Q_LEN,
						&mailbox);
			}
		}
		if (flag_of_tx_1 == 1 && flag_of_rx_1 == 1) {
			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0) {
				flag_of_tx_1 = flag_of_rx_1 = 0;
				ProcessQueue(&hcan2, &queue3, TxHdrs3, TxDats3, Q_LEN,
						&mailbox1);
			}
		}

		CheckBusIdleAndRestart();
		osDelay(1);
	}
}

void SaveMotorToSlot(uint8_t motor_idx, uint8_t slot) {
	if (slot >= NUM_SLOTS || motor_idx >= NUM_MOTORS)
		return;

	MotorSlot_t tmp;
	switch (motor_idx) {
	case 0: // 1_1
		tmp.kp_angle = kp_angle_1_1;
		tmp.ki_angle = ki_angle_1_1;
		tmp.kd_angle = kd_angle_1_1;
		tmp.kp_speed = kp_speed_1_1;
		tmp.ki_speed = ki_speed_1_1;
		tmp.kp_curr = kp_curr_1_1;
		tmp.ki_curr = ki_curr_1_1;
		tmp.obstacle = obstacle1_1;
		tmp.speed_limit = speed_limit1_1;
		tmp.phase = phase1_1;
		tmp.set_zero = set_zero;
		break;
	case 1: // 1_2
		tmp.kp_angle = kp_angle_1_2;
		tmp.ki_angle = ki_angle_1_2;
		tmp.kd_angle = kd_angle_1_2;
		tmp.kp_speed = kp_speed_1_2;
		tmp.ki_speed = ki_speed_1_2;
		tmp.kp_curr = kp_curr_1_2;
		tmp.ki_curr = ki_curr_1_2;
		tmp.obstacle = obstacle1_2;
		tmp.speed_limit = speed_limit1_2;
		tmp.phase = phase1_2;
		tmp.set_zero = set_zero;
		break;
	case 2: // 2_1
		tmp.kp_angle = kp_angle_2_1;
		tmp.ki_angle = ki_angle_2_1;
		tmp.kd_angle = kd_angle_2_1;
		tmp.kp_speed = kp_speed_2_1;
		tmp.ki_speed = ki_speed_2_1;
		tmp.kp_curr = kp_curr_2_1;
		tmp.ki_curr = ki_curr_2_1;
		tmp.obstacle = obstacle2_1;
		tmp.speed_limit = speed_limit2_1;
		tmp.phase = phase2_1;
		tmp.set_zero = set_zero;
		break;
	case 3: // 2_2
		tmp.kp_angle = kp_angle_2_2;
		tmp.ki_angle = ki_angle_2_2;
		tmp.kd_angle = kd_angle_2_2;
		tmp.kp_speed = kp_speed_2_2;
		tmp.ki_speed = ki_speed_2_2;
		tmp.kp_curr = kp_curr_2_2;
		tmp.ki_curr = ki_curr_2_2;
		tmp.obstacle = obstacle2_2;
		tmp.speed_limit = speed_limit2_2;
		tmp.phase = phase2_2;
		tmp.set_zero = set_zero;
		break;
	case 4: // 3_1
		tmp.kp_angle = kp_angle_3_1;
		tmp.ki_angle = ki_angle_3_1;
		tmp.kd_angle = kd_angle_3_1;
		tmp.kp_speed = kp_speed_3_1;
		tmp.ki_speed = ki_speed_3_1;
		tmp.kp_curr = kp_curr_3_1;
		tmp.ki_curr = ki_curr_3_1;
		tmp.obstacle = obstacle3_1;
		tmp.speed_limit = speed_limit3_1;
		tmp.phase = phase3_1;
		tmp.set_zero = set_zero;
		break;
	case 5: // 3_2
		tmp.kp_angle = kp_angle_3_2;
		tmp.ki_angle = ki_angle_3_2;
		tmp.kd_angle = kd_angle_3_2;
		tmp.kp_speed = kp_speed_3_2;
		tmp.ki_speed = ki_speed_3_2;
		tmp.kp_curr = kp_curr_3_2;
		tmp.ki_curr = ki_curr_3_2;
		tmp.obstacle = obstacle3_2;
		tmp.speed_limit = speed_limit3_2;
		tmp.phase = phase3_2;
		tmp.set_zero = set_zero;
		break;
	default:
		return;
	}

	slots[slot][motor_idx] = tmp;
	slot_initialized[slot][motor_idx] = true;
}
void LoadMotorFromSlot(uint8_t motor_idx, uint8_t slot) {
	MotorSlot_t tmp;

	if (slot >= NUM_SLOTS || motor_idx >= NUM_MOTORS)
		return;

	if (slot_initialized[slot][motor_idx]) {
		tmp = slots[slot][motor_idx];
	} else {
		tmp = slot_template[slot][motor_idx];
	}
	switch (motor_idx) {
	case 0:
		kp_angle_1_1 = tmp.kp_angle;
		ki_angle_1_1 = tmp.ki_angle;
		kd_angle_1_1 = tmp.kd_angle;
		kp_speed_1_1 = tmp.kp_speed;
		ki_speed_1_1 = tmp.ki_speed;
		kp_curr_1_1 = tmp.kp_curr;
		ki_curr_1_1 = tmp.ki_curr;
		obstacle1_1 = tmp.obstacle;
		speed_limit1_1 = tmp.speed_limit;
		phase1_1 = tmp.phase;
		set_zero = tmp.set_zero;
		break;
	case 1:
		kp_angle_1_2 = tmp.kp_angle;
		ki_angle_1_2 = tmp.ki_angle;
		kd_angle_1_2 = tmp.kd_angle;
		kp_speed_1_2 = tmp.kp_speed;
		ki_speed_1_2 = tmp.ki_speed;
		kp_curr_1_2 = tmp.kp_curr;
		ki_curr_1_2 = tmp.ki_curr;
		obstacle1_2 = tmp.obstacle;
		speed_limit1_2 = tmp.speed_limit;
		phase1_2 = tmp.phase;
		set_zero = tmp.set_zero;
		break;
	case 2:
		kp_angle_2_1 = tmp.kp_angle;
		ki_angle_2_1 = tmp.ki_angle;
		kd_angle_2_1 = tmp.kd_angle;
		kp_speed_2_1 = tmp.kp_speed;
		ki_speed_2_1 = tmp.ki_speed;
		kp_curr_2_1 = tmp.kp_curr;
		ki_curr_2_1 = tmp.ki_curr;
		obstacle2_1 = tmp.obstacle;
		speed_limit2_1 = tmp.speed_limit;
		phase2_1 = tmp.phase;
		set_zero = tmp.set_zero;
		break;
	case 3:
		kp_angle_2_2 = tmp.kp_angle;
		ki_angle_2_2 = tmp.ki_angle;
		kd_angle_2_2 = tmp.kd_angle;
		kp_speed_2_2 = tmp.kp_speed;
		ki_speed_2_2 = tmp.ki_speed;
		kp_curr_2_2 = tmp.kp_curr;
		ki_curr_2_2 = tmp.ki_curr;
		obstacle2_2 = tmp.obstacle;
		speed_limit2_2 = tmp.speed_limit;
		phase2_2 = tmp.phase;
		set_zero = tmp.set_zero;
		break;
	case 4:
		kp_angle_3_1 = tmp.kp_angle;
		ki_angle_3_1 = tmp.ki_angle;
		kd_angle_3_1 = tmp.kd_angle;
		kp_speed_3_1 = tmp.kp_speed;
		ki_speed_3_1 = tmp.ki_speed;
		kp_curr_3_1 = tmp.kp_curr;
		ki_curr_3_1 = tmp.ki_curr;
		obstacle3_1 = tmp.obstacle;
		speed_limit3_1 = tmp.speed_limit;
		phase3_1 = tmp.phase;
		set_zero = tmp.set_zero;
		break;
	case 5:
		kp_angle_3_2 = tmp.kp_angle;
		ki_angle_3_2 = tmp.ki_angle;
		kd_angle_3_2 = tmp.kd_angle;
		kp_speed_3_2 = tmp.kp_speed;
		ki_speed_3_2 = tmp.ki_speed;
		kp_curr_3_2 = tmp.kp_curr;
		ki_curr_3_2 = tmp.ki_curr;
		obstacle3_2 = tmp.obstacle;
		speed_limit3_2 = tmp.speed_limit;
		phase3_2 = tmp.phase;
		set_zero = tmp.set_zero;
		break;
	default:
		break;
	}
}
void LoadAllCurrentSlots(void) {
	LoadMotorFromSlot(0, system_type1_1);
	LoadMotorFromSlot(1, system_type1_2);
	LoadMotorFromSlot(2, system_type2_1);
	LoadMotorFromSlot(3, system_type2_2);
	LoadMotorFromSlot(4, system_type3_1);
	LoadMotorFromSlot(5, system_type3_2);
}
void SaveAllCurrentSlots(void) {
	SaveMotorToSlot(0, system_type1_1);
	SaveMotorToSlot(1, system_type1_2);
	SaveMotorToSlot(2, system_type2_1);
	SaveMotorToSlot(3, system_type2_2);
	SaveMotorToSlot(4, system_type3_1);
	SaveMotorToSlot(5, system_type3_2);
}
void InitSlotsFromTemplate(void) {
	for (uint8_t s = 0; s < NUM_SLOTS; ++s) {
		for (uint8_t m = 0; m < NUM_MOTORS; ++m) {
			slots[s][m] = slot_template[s][m];
			slot_initialized[s][m] = true;
		}
	}
}

void reset_all_variables_to_zero(void) {
	set_angle1_1 = set_angle1_2 = set_angle2_1 = set_angle2_2 = set_angle3_1 =
			set_angle3_2 = 0;
	angle1_1 = angle1_2 = angle2_1 = angle2_2 = angle3_1 = angle3_2 = 0;
	speed1_1 = speed1_2 = speed2_1 = speed2_2 = speed3_1 = speed3_2 = 0;
	current1_1 = current1_2 = current2_1 = current2_2 = current3_1 =
			current3_2 = 0;
	obstacle1_1 = obstacle1_2 = obstacle2_1 = obstacle2_2 = obstacle3_1 =
			obstacle3_2 = 0;
	speed_limit1_1 = speed_limit1_2 = speed_limit2_1 = speed_limit2_2 =
			speed_limit3_1 = speed_limit3_2 = 85;
	phase1_1 = phase1_2 = phase2_1 = phase2_2 = phase3_1 = phase3_2 = 0;
	kp_angle_1_1 = kp_angle_1_2 = kp_angle_2_1 = kp_angle_2_2 = kp_angle_3_1 =
			kp_angle_3_2 = 0;
	ki_angle_1_1 = ki_angle_1_2 = ki_angle_2_1 = ki_angle_2_2 = ki_angle_3_1 =
			ki_angle_3_2 = 0;
	kd_angle_1_1 = kd_angle_1_2 = kd_angle_2_1 = kd_angle_2_2 = kd_angle_3_1 =
			kd_angle_3_2 = 0;
	kp_speed_1_1 = kp_speed_1_2 = kp_speed_2_1 = kp_speed_2_2 = kp_speed_3_1 =
			kp_speed_3_2 = 0;
	ki_speed_1_1 = ki_speed_1_2 = ki_speed_2_1 = ki_speed_2_2 = ki_speed_3_1 =
			ki_speed_3_2 = 0;
	kp_curr_1_1 = kp_curr_1_2 = kp_curr_2_1 = kp_curr_2_2 = kp_curr_3_1 =
			kp_curr_3_2 = 0;
	ki_curr_1_1 = ki_curr_1_2 = ki_curr_2_1 = ki_curr_2_2 = ki_curr_3_1 =
			ki_curr_3_2 = 0;
}

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
