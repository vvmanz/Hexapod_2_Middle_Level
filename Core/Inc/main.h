/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <w5500/socket.h>
#include <w5500/loopback.h>
#include "spi.h"
#include <stdbool.h>

#define BUS_IDLE_TIMEOUT_MS  100
#define Q_LEN 9
#define Q_LEN_1 1
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//--------------Odometry----------------//


extern uint32_t distance_mm ;
extern int32_t flow_dx;
extern int32_t flow_dy;
extern uint8_t brightness;
extern uint32_t last_time_us;


// ---------Ethernet-----------//
extern void W5500_Init();
//-----------CAN----------------//

// -----------Slot System Control -------//
#define NUM_SLOTS    7
#define NUM_MOTORS   6
typedef struct {
	int32_t kp_angle;
	int32_t ki_angle;
	int32_t kd_angle;

	int32_t kp_speed;
	int32_t ki_speed;

	int32_t kp_curr;
	int32_t ki_curr;

	int16_t obstacle;
	int8_t speed_limit;
	int8_t phase;

	int8_t set_zero;
} MotorSlot_t;
void reset_all_variables_to_zero(void);


extern volatile uint32_t lastActivityTimestamp, lastActivityTimestamp1;
extern uint8_t queue1, queue2, queue3;

extern uint32_t error_can, errors_count, packets_count, mailbox, mailbox1,
		received_call;

extern uint8_t TxDataAngles1[8], TxDataOther1[8], TxDataCoeffs1_1[8],
		TxDataCoeffs1_2[8], TxDataCoeffs1_3[8], TxDataCoeffs1_4[8],
		TxDataCoeffs1_5[8], TxDataCoeffs1_6[8], TxDataCoeffs1_7[8];

extern uint8_t TxDataAngles2[8], TxDataOther2[8], TxDataCoeffs2_1[8],
		TxDataCoeffs2_2[8], TxDataCoeffs2_3[8], TxDataCoeffs2_4[8],
		TxDataCoeffs2_5[8], TxDataCoeffs2_6[8], TxDataCoeffs2_7[8];

extern uint8_t TxDataAngles3[8], TxDataOther3[8], TxDataCoeffs3_1[8],
		TxDataCoeffs3_2[8], TxDataCoeffs3_3[8], TxDataCoeffs3_4[8],
		TxDataCoeffs3_5[8], TxDataCoeffs3_6[8], TxDataCoeffs3_7[8];

extern CAN_TxHeaderTypeDef TxAngles1, TxAngles2, TxAngles3, TxOther1, TxOther2,
		TxOther3, TxCoeffs1_1, TxCoeffs1_2, TxCoeffs1_3, TxCoeffs1_4,
		TxCoeffs1_5, TxCoeffs1_6, TxCoeffs1_7, TxCoeffs2_1, TxCoeffs2_2,
		TxCoeffs2_3, TxCoeffs2_4, TxCoeffs2_5, TxCoeffs2_6, TxCoeffs2_7,
		TxCoeffs3_1, TxCoeffs3_2, TxCoeffs3_3, TxCoeffs3_4, TxCoeffs3_5,
		TxCoeffs3_6, TxCoeffs3_7;

extern uint8_t RxAngles1[8], RxAngles2[8], RxAngles3[8], RxSpeeds1[8],
		RxSpeeds2[8], RxSpeeds3[8], RxCurrents1[8], RxCurrents2[8],
		RxCurrents3[8];

extern CAN_TxHeaderTypeDef *const TxHdrs1[Q_LEN], *const TxHdrs2[Q_LEN],
		*const TxHdrs3[Q_LEN];
extern uint8_t *const TxDats1[Q_LEN], *const TxDats2[Q_LEN],
		*const TxDats3[Q_LEN];

extern CAN_RxHeaderTypeDef RxHeader, RxHeader1, RxHeader2;
extern uint8_t RxData[8], RxData1[8], RxData2[8];

extern volatile uint8_t flag_of_tx, flag_of_rx, flag_of_tx_1, flag_of_rx_1;
// --------------Motor Control Stats-----------------//
extern int32_t set_angle1_1, set_angle1_2, set_angle2_1, set_angle2_2,
		set_angle3_1, set_angle3_2;

extern int32_t angle1_1, angle1_2, angle2_1, angle2_2, angle3_1, angle3_2,
		speed1_1, speed1_2, speed2_1, speed2_2, speed3_1, speed3_2, current1_1,
		current1_2, current2_1, current2_2, current3_1, current3_2;

extern int16_t obstacle1_1, obstacle1_2, obstacle2_1, obstacle2_2, obstacle3_1,
		obstacle3_2;
extern uint8_t speed_limit1_1, speed_limit1_2, speed_limit2_1, speed_limit2_2,
		speed_limit3_1, speed_limit3_2;
extern int8_t phase1_1, phase1_2, phase2_1, phase2_2, phase3_1, phase3_2;

extern int8_t set_zero;
extern int8_t system_type1_1, system_type1_2, system_type2_1, system_type2_2,
		system_type3_1, system_type3_2;

extern int32_t kp_angle_1_1, kp_angle_1_2, kp_angle_2_1, kp_angle_2_2,
		kp_angle_3_1, kp_angle_3_2;
extern int32_t ki_angle_1_1, ki_angle_1_2, ki_angle_2_1, ki_angle_2_2,
		ki_angle_3_1, ki_angle_3_2;
extern int32_t kd_angle_1_1, kd_angle_1_2, kd_angle_2_1, kd_angle_2_2,
		kd_angle_3_1, kd_angle_3_2;
extern int32_t kp_speed_1_1, kp_speed_1_2, kp_speed_2_1, kp_speed_2_2,
		kp_speed_3_1, kp_speed_3_2;
extern int32_t ki_speed_1_1, ki_speed_1_2, ki_speed_2_1, ki_speed_2_2,
		ki_speed_3_1, ki_speed_3_2;
extern int32_t kp_curr_1_1, kp_curr_1_2, kp_curr_2_1, kp_curr_2_2, kp_curr_3_1,
		kp_curr_3_2;
extern int32_t ki_curr_1_1, ki_curr_1_2, ki_curr_2_1, ki_curr_2_2, ki_curr_3_1,
		ki_curr_3_2;

extern int32_t angle1_1, angle1_2, angle2_1, angle2_2, angle3_1, angle3_2,
		speed1_1, speed1_2, speed2_1, speed2_2, speed3_1, speed3_2, current1_1,
		current1_2, current2_1, current2_2, current3_1, current3_2;

//--------------ROS------------//
extern void prepare_frames();
extern int32_t ConvertBytesToInt32(uint8_t *data);

extern void Prepare_set_angles();
extern void Prepare_set_koeffs();
extern void Set_initial_koeffs();

extern void StartRosTask(void *argument);
extern void StartCanTask(void *argument);
extern void StartOdometryTask(void *argument);

extern volatile uint8_t ros_new_coeffs_flag;

extern void BridgeDecode(int a);
extern void CheckBusIdleAndRestart();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define SPI1_RESET_Pin GPIO_PIN_4
#define SPI1_RESET_GPIO_Port GPIOC
#define SPI1_INT_Pin GPIO_PIN_5
#define SPI1_INT_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
