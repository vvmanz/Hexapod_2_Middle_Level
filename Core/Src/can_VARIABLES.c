#include "main.h"

volatile uint32_t lastActivityTimestamp = 0;
volatile uint32_t lastActivityTimestamp1 = 0;

uint8_t queue1, queue2, queue3;

uint32_t error_can = 0;
uint32_t errors_count, packets_count;
uint32_t mailbox, mailbox1;

uint32_t received_call;

uint8_t TxDataAngles1[8] = { 0, };
uint8_t TxDataOther1[8] = { 0, };
uint8_t TxDataCoeffs1_1[8] = { 0, };
uint8_t TxDataCoeffs1_2[8] = { 0, };
uint8_t TxDataCoeffs1_3[8] = { 0, };
uint8_t TxDataCoeffs1_4[8] = { 0, };
uint8_t TxDataCoeffs1_5[8] = { 0, };
uint8_t TxDataCoeffs1_6[8] = { 0, };
uint8_t TxDataCoeffs1_7[8] = { 0, };

uint8_t TxDataAngles2[8] = { 0, };
uint8_t TxDataOther2[8] = { 0, };
uint8_t TxDataCoeffs2_1[8] = { 0, };
uint8_t TxDataCoeffs2_2[8] = { 0, };
uint8_t TxDataCoeffs2_3[8] = { 0, };
uint8_t TxDataCoeffs2_4[8] = { 0, };
uint8_t TxDataCoeffs2_5[8] = { 0, };
uint8_t TxDataCoeffs2_6[8] = { 0, };
uint8_t TxDataCoeffs2_7[8] = { 0, };

uint8_t TxDataAngles3[8] = { 0, };
uint8_t TxDataOther3[8] = { 0, };
uint8_t TxDataCoeffs3_1[8] = { 0, };
uint8_t TxDataCoeffs3_2[8] = { 0, };
uint8_t TxDataCoeffs3_3[8] = { 0, };
uint8_t TxDataCoeffs3_4[8] = { 0, };
uint8_t TxDataCoeffs3_5[8] = { 0, };
uint8_t TxDataCoeffs3_6[8] = { 0, };
uint8_t TxDataCoeffs3_7[8] = { 0, };

CAN_TxHeaderTypeDef TxAngles1;
CAN_TxHeaderTypeDef TxAngles2;
CAN_TxHeaderTypeDef TxAngles3;

CAN_TxHeaderTypeDef TxOther1;
CAN_TxHeaderTypeDef TxOther2;
CAN_TxHeaderTypeDef TxOther3;

CAN_TxHeaderTypeDef TxCoeffs1_1;
CAN_TxHeaderTypeDef TxCoeffs1_2;
CAN_TxHeaderTypeDef TxCoeffs1_3;
CAN_TxHeaderTypeDef TxCoeffs1_4;
CAN_TxHeaderTypeDef TxCoeffs1_5;
CAN_TxHeaderTypeDef TxCoeffs1_6;
CAN_TxHeaderTypeDef TxCoeffs1_7;

CAN_TxHeaderTypeDef TxCoeffs2_1;
CAN_TxHeaderTypeDef TxCoeffs2_2;
CAN_TxHeaderTypeDef TxCoeffs2_3;
CAN_TxHeaderTypeDef TxCoeffs2_4;
CAN_TxHeaderTypeDef TxCoeffs2_5;
CAN_TxHeaderTypeDef TxCoeffs2_6;
CAN_TxHeaderTypeDef TxCoeffs2_7;

CAN_TxHeaderTypeDef TxCoeffs3_1;
CAN_TxHeaderTypeDef TxCoeffs3_2;
CAN_TxHeaderTypeDef TxCoeffs3_3;
CAN_TxHeaderTypeDef TxCoeffs3_4;
CAN_TxHeaderTypeDef TxCoeffs3_5;
CAN_TxHeaderTypeDef TxCoeffs3_6;
CAN_TxHeaderTypeDef TxCoeffs3_7;

uint8_t RxAngles1[8] = { 0, };
uint8_t RxAngles2[8] = { 0, };
uint8_t RxAngles3[8] = { 0, };

uint8_t RxSpeeds1[8] = { 0, };
uint8_t RxSpeeds2[8] = { 0, };
uint8_t RxSpeeds3[8] = { 0, };

uint8_t RxCurrents1[8] = { 0, };
uint8_t RxCurrents2[8] = { 0, };
uint8_t RxCurrents3[8] = { 0, };

CAN_RxHeaderTypeDef RxHeader;
CAN_RxHeaderTypeDef RxHeader1;
CAN_RxHeaderTypeDef RxHeader2;
uint8_t RxData[8] = { 0, };
uint8_t RxData1[8] = { 0, };
uint8_t RxData2[8] = { 0, };

volatile uint8_t flag_of_tx = 0;
volatile uint8_t flag_of_rx = 0;
volatile uint8_t flag_of_tx_1 = 0;
volatile uint8_t flag_of_rx_1 = 0;

void Set_initial_koeffs() {
	kp_angle_1_1 = kp_angle_1_2 = kp_angle_2_1 = kp_angle_2_2 = kp_angle_3_1 =
			kp_angle_3_2 = 4 * 1000;
	ki_angle_1_1 = ki_angle_1_2 = ki_angle_2_1 = ki_angle_2_2 = ki_angle_3_1 =
			ki_angle_3_2 = 4 * 1000;
	kd_angle_1_1 = kd_angle_1_2 = kd_angle_2_1 = kd_angle_2_2 = kd_angle_3_1 =
			kd_angle_3_2 = 10;

	kp_speed_1_1 = kp_speed_1_2 = kp_speed_2_1 = kp_speed_2_2 = kp_speed_3_1 =
			kp_speed_3_2 = 5;
	ki_speed_1_1 = ki_speed_1_2 = ki_speed_2_1 = ki_speed_2_2 = ki_speed_3_1 =
			ki_speed_3_2 = 5;

	kp_curr_1_1 = kp_curr_1_2 = kp_curr_2_1 = kp_curr_2_2 = kp_curr_3_1 =
			kp_curr_3_2 = 50;
	ki_curr_1_1 = ki_curr_1_2 = ki_curr_2_1 = ki_curr_2_2 = ki_curr_3_1 =
			ki_curr_3_2 = 3000;

	phase1_1 = phase1_2 = phase2_1 = phase2_2 = phase3_1 = phase3_2 = 0;
	set_zero = 0;
	system_type1_1 = set_angle1_2 = system_type2_1 = system_type2_2 =
			system_type3_1 = system_type3_2 = 0;

	obstacle1_1 = obstacle1_2 = obstacle2_1 = obstacle2_2 = obstacle3_1 =
			obstacle3_2 = 0;
	speed_limit1_1 = speed_limit1_2 = speed_limit2_1 = speed_limit2_2 =
			speed_limit3_1 = speed_limit3_2 = 1;

}

void prepare_frames() {
	TxAngles1.StdId = 0x0200;
	TxAngles1.ExtId = 0;
	TxAngles1.RTR = CAN_RTR_DATA;
	TxAngles1.IDE = CAN_ID_STD;
	TxAngles1.DLC = 8;
	TxAngles1.TransmitGlobalTime = 0;

	TxAngles2.StdId = 0x0300;
	TxAngles2.ExtId = 0;
	TxAngles2.RTR = CAN_RTR_DATA;
	TxAngles2.IDE = CAN_ID_STD;
	TxAngles2.DLC = 8;
	TxAngles2.TransmitGlobalTime = 0;

	TxAngles3.StdId = 0x0400;
	TxAngles3.ExtId = 0;
	TxAngles3.RTR = CAN_RTR_DATA;
	TxAngles3.IDE = CAN_ID_STD;
	TxAngles3.DLC = 8;
	TxAngles3.TransmitGlobalTime = 0;

	TxOther1.StdId = 0x0201;
	TxOther1.ExtId = 0;
	TxOther1.RTR = CAN_RTR_DATA;
	TxOther1.IDE = CAN_ID_STD;
	TxOther1.DLC = 8;
	TxOther1.TransmitGlobalTime = 0;

	TxOther2.StdId = 0x0301;
	TxOther2.ExtId = 0;
	TxOther2.RTR = CAN_RTR_DATA;
	TxOther2.IDE = CAN_ID_STD;
	TxOther2.DLC = 8;
	TxOther2.TransmitGlobalTime = 0;

	TxOther3.StdId = 0x0401;
	TxOther3.ExtId = 0;
	TxOther3.RTR = CAN_RTR_DATA;
	TxOther3.IDE = CAN_ID_STD;
	TxOther3.DLC = 8;
	TxOther3.TransmitGlobalTime = 0;

	TxCoeffs1_1.StdId = 0x0202;
	TxCoeffs1_1.ExtId = 0;
	TxCoeffs1_1.RTR = CAN_RTR_DATA;
	TxCoeffs1_1.IDE = CAN_ID_STD;
	TxCoeffs1_1.DLC = 8;
	TxCoeffs1_1.TransmitGlobalTime = 0;

	TxCoeffs2_1.StdId = 0x0302;
	TxCoeffs2_1.ExtId = 0;
	TxCoeffs2_1.RTR = CAN_RTR_DATA;
	TxCoeffs2_1.IDE = CAN_ID_STD;
	TxCoeffs2_1.DLC = 8;
	TxCoeffs2_1.TransmitGlobalTime = 0;

	TxCoeffs3_1.StdId = 0x0402;
	TxCoeffs3_1.ExtId = 0;
	TxCoeffs3_1.RTR = CAN_RTR_DATA;
	TxCoeffs3_1.IDE = CAN_ID_STD;
	TxCoeffs3_1.DLC = 8;
	TxCoeffs3_1.TransmitGlobalTime = 0;

	TxCoeffs1_2.StdId = 0x0203;
	TxCoeffs1_2.ExtId = 0;
	TxCoeffs1_2.RTR = CAN_RTR_DATA;
	TxCoeffs1_2.IDE = CAN_ID_STD;
	TxCoeffs1_2.DLC = 8;
	TxCoeffs1_2.TransmitGlobalTime = 0;

	TxCoeffs2_2.StdId = 0x0303;
	TxCoeffs2_2.ExtId = 0;
	TxCoeffs2_2.RTR = CAN_RTR_DATA;
	TxCoeffs2_2.IDE = CAN_ID_STD;
	TxCoeffs2_2.DLC = 8;
	TxCoeffs2_2.TransmitGlobalTime = 0;

	TxCoeffs3_2.StdId = 0x0403;
	TxCoeffs3_2.ExtId = 0;
	TxCoeffs3_2.RTR = CAN_RTR_DATA;
	TxCoeffs3_2.IDE = CAN_ID_STD;
	TxCoeffs3_2.DLC = 8;
	TxCoeffs3_2.TransmitGlobalTime = 0;

	TxCoeffs1_3.StdId = 0x0204;
	TxCoeffs1_3.ExtId = 0;
	TxCoeffs1_3.RTR = CAN_RTR_DATA;
	TxCoeffs1_3.IDE = CAN_ID_STD;
	TxCoeffs1_3.DLC = 8;
	TxCoeffs1_3.TransmitGlobalTime = 0;

	TxCoeffs2_3.StdId = 0x0304;
	TxCoeffs2_3.ExtId = 0;
	TxCoeffs2_3.RTR = CAN_RTR_DATA;
	TxCoeffs2_3.IDE = CAN_ID_STD;
	TxCoeffs2_3.DLC = 8;
	TxCoeffs2_3.TransmitGlobalTime = 0;

	TxCoeffs3_3.StdId = 0x0404;
	TxCoeffs3_3.ExtId = 0;
	TxCoeffs3_3.RTR = CAN_RTR_DATA;
	TxCoeffs3_3.IDE = CAN_ID_STD;
	TxCoeffs3_3.DLC = 8;
	TxCoeffs3_3.TransmitGlobalTime = 0;

	TxCoeffs1_4.StdId = 0x0205;
	TxCoeffs1_4.ExtId = 0;
	TxCoeffs1_4.RTR = CAN_RTR_DATA;
	TxCoeffs1_4.IDE = CAN_ID_STD;
	TxCoeffs1_4.DLC = 8;
	TxCoeffs1_4.TransmitGlobalTime = 0;

	TxCoeffs2_4.StdId = 0x0305;
	TxCoeffs2_4.ExtId = 0;
	TxCoeffs2_4.RTR = CAN_RTR_DATA;
	TxCoeffs2_4.IDE = CAN_ID_STD;
	TxCoeffs2_4.DLC = 8;
	TxCoeffs2_4.TransmitGlobalTime = 0;

	TxCoeffs3_4.StdId = 0x0405;
	TxCoeffs3_4.ExtId = 0;
	TxCoeffs3_4.RTR = CAN_RTR_DATA;
	TxCoeffs3_4.IDE = CAN_ID_STD;
	TxCoeffs3_4.DLC = 8;
	TxCoeffs3_4.TransmitGlobalTime = 0;

	TxCoeffs1_5.StdId = 0x0206;
	TxCoeffs1_5.ExtId = 0;
	TxCoeffs1_5.RTR = CAN_RTR_DATA;
	TxCoeffs1_5.IDE = CAN_ID_STD;
	TxCoeffs1_5.DLC = 8;
	TxCoeffs1_5.TransmitGlobalTime = 0;

	TxCoeffs2_5.StdId = 0x0306;
	TxCoeffs2_5.ExtId = 0;
	TxCoeffs2_5.RTR = CAN_RTR_DATA;
	TxCoeffs2_5.IDE = CAN_ID_STD;
	TxCoeffs2_5.DLC = 8;
	TxCoeffs2_5.TransmitGlobalTime = 0;

	TxCoeffs3_5.StdId = 0x0406;
	TxCoeffs3_5.ExtId = 0;
	TxCoeffs3_5.RTR = CAN_RTR_DATA;
	TxCoeffs3_5.IDE = CAN_ID_STD;
	TxCoeffs3_5.DLC = 8;
	TxCoeffs3_5.TransmitGlobalTime = 0;

	TxCoeffs1_6.StdId = 0x0207;
	TxCoeffs1_6.ExtId = 0;
	TxCoeffs1_6.RTR = CAN_RTR_DATA;
	TxCoeffs1_6.IDE = CAN_ID_STD;
	TxCoeffs1_6.DLC = 8;
	TxCoeffs1_6.TransmitGlobalTime = 0;

	TxCoeffs2_6.StdId = 0x0307;
	TxCoeffs2_6.ExtId = 0;
	TxCoeffs2_6.RTR = CAN_RTR_DATA;
	TxCoeffs2_6.IDE = CAN_ID_STD;
	TxCoeffs2_6.DLC = 8;
	TxCoeffs2_6.TransmitGlobalTime = 0;

	TxCoeffs3_6.StdId = 0x0407;
	TxCoeffs3_6.ExtId = 0;
	TxCoeffs3_6.RTR = CAN_RTR_DATA;
	TxCoeffs3_6.IDE = CAN_ID_STD;
	TxCoeffs3_6.DLC = 8;
	TxCoeffs3_6.TransmitGlobalTime = 0;

	TxCoeffs1_7.StdId = 0x0208;
	TxCoeffs1_7.ExtId = 0;
	TxCoeffs1_7.RTR = CAN_RTR_DATA;
	TxCoeffs1_7.IDE = CAN_ID_STD;
	TxCoeffs1_7.DLC = 8;
	TxCoeffs1_7.TransmitGlobalTime = 0;

	TxCoeffs2_7.StdId = 0x0308;
	TxCoeffs2_7.ExtId = 0;
	TxCoeffs2_7.RTR = CAN_RTR_DATA;
	TxCoeffs2_7.IDE = CAN_ID_STD;
	TxCoeffs2_7.DLC = 8;
	TxCoeffs2_7.TransmitGlobalTime = 0;

	TxCoeffs3_7.StdId = 0x0408;
	TxCoeffs3_7.ExtId = 0;
	TxCoeffs3_7.RTR = CAN_RTR_DATA;
	TxCoeffs3_7.IDE = CAN_ID_STD;
	TxCoeffs3_7.DLC = 8;
	TxCoeffs3_7.TransmitGlobalTime = 0;
}
