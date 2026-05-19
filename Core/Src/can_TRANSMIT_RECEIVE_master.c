#include "main.h"
#include "can.h"
#include <string.h>

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	if (hcan->Instance == CAN1) {
		lastActivityTimestamp = HAL_GetTick();
		error_can = HAL_CAN_GetError(&hcan1);
		errors_count++;
	}
	if (hcan->Instance == CAN2) {
		lastActivityTimestamp1 = HAL_GetTick();
		error_can = HAL_CAN_GetError(&hcan2);
		errors_count++;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	uint32_t now = HAL_GetTick();
	if (hcan->Instance == CAN1) {
		lastActivityTimestamp = now;
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData)
				== HAL_OK) {
			if (RxHeader.StdId == 0x0105) {
				packets_count++;
				flag_of_rx = 1;
				memcpy(RxAngles1, RxData, sizeof(RxAngles1));
				angle1_1 = ConvertBytesToInt32(RxAngles1);
				angle1_2 = ConvertBytesToInt32(&RxAngles1[4]);
			}

			if (RxHeader.StdId == 0x0103) {
				packets_count++;
				flag_of_rx = 1;
				memcpy(RxSpeeds1, RxData, sizeof(RxSpeeds1));
				speed1_1 = ConvertBytesToInt32(RxSpeeds1);
				speed1_2 = ConvertBytesToInt32(&RxSpeeds1[4]);
			}

			if (RxHeader.StdId == 0x0101) {
				packets_count++;
				flag_of_rx = 1;
				memcpy(RxCurrents1, RxData, sizeof(RxCurrents1));
				current1_1 = ConvertBytesToInt32(RxCurrents1);
				current1_2 = ConvertBytesToInt32(&RxCurrents1[4]);
			}
			received_call++;
		}
	}
	if (hcan->Instance == CAN2) {
		lastActivityTimestamp1 = now;
		if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader2, RxData2)
				== HAL_OK) {
			if (RxHeader2.StdId == 0x0505) {
				packets_count++;
				flag_of_rx_1 = 1;
				memcpy(RxAngles3, RxData2, sizeof(RxAngles3));
				angle3_1 = ConvertBytesToInt32(RxAngles3);
				angle3_2 = ConvertBytesToInt32(&RxAngles3[4]);
			}
			if (RxHeader2.StdId == 0x0503) {
				packets_count++;
				flag_of_rx_1 = 1;
				memcpy(RxSpeeds3, RxData2, sizeof(RxSpeeds3));
				speed3_1 = ConvertBytesToInt32(RxSpeeds3);
				speed3_2 = ConvertBytesToInt32(&RxSpeeds3[4]);
			}
			if (RxHeader2.StdId == 0x0501) {
				packets_count++;
				flag_of_rx_1 = 1;
				memcpy(RxCurrents3, RxData2, sizeof(RxCurrents2));
				current3_1 = ConvertBytesToInt32(RxCurrents3);
				current3_2 = ConvertBytesToInt32(&RxCurrents3[4]);
			}
			received_call++;
		}
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	uint32_t now = HAL_GetTick();
	lastActivityTimestamp = now;
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &RxHeader1, RxData1)
			== HAL_OK) {
		if (RxHeader1.StdId == 0x0104) {
			packets_count++;
			flag_of_rx = 2;
			memcpy(RxAngles2, RxData1, sizeof(RxAngles2));
			angle2_1 = ConvertBytesToInt32(RxAngles2);
			angle2_2 = ConvertBytesToInt32(&RxAngles2[4]);

		}
		if (RxHeader1.StdId == 0x0102) {
			packets_count++;
			flag_of_rx = 2;
			memcpy(RxSpeeds2, RxData1, sizeof(RxSpeeds2));
			speed2_1 = ConvertBytesToInt32(RxSpeeds2);
			speed2_2 = ConvertBytesToInt32(&RxSpeeds2[4]);

		}
		if (RxHeader1.StdId == 0x0100) {
			packets_count++;
			flag_of_rx = 2;
			memcpy(RxCurrents2, RxData1, sizeof(RxCurrents2));
			current2_1 = ConvertBytesToInt32(RxCurrents2);
			current2_2 = ConvertBytesToInt32(&RxCurrents2[4]);
		}
		received_call++;
	}
}

void CheckBusIdleAndRestart() {
	uint32_t now = HAL_GetTick();
	if ((now - lastActivityTimestamp) > BUS_IDLE_TIMEOUT_MS) {
		flag_of_tx = 0;
		flag_of_rx = 0;
		queue1 = queue2 = 0;
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
			HAL_CAN_AddTxMessage(&hcan1, &TxAngles1, TxDataAngles1, &mailbox);
			lastActivityTimestamp = now;
		}
	}
	if ((now - lastActivityTimestamp1) > BUS_IDLE_TIMEOUT_MS) {
		flag_of_tx_1 = 0;
		flag_of_rx_1 = 0;
		queue3 = 0;
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0) {
			HAL_CAN_AddTxMessage(&hcan2, &TxAngles3, TxDataAngles3, &mailbox1);
			lastActivityTimestamp1 = now;
		}
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	if (hcan->Instance == CAN1) {
		flag_of_tx = 1;
		lastActivityTimestamp = HAL_GetTick();
	}
	if (hcan->Instance == CAN2) {
		flag_of_tx_1 = 1;
		lastActivityTimestamp1 = HAL_GetTick();
	}
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
	if (hcan->Instance == CAN1) {
		flag_of_tx = 1;
		lastActivityTimestamp = HAL_GetTick();
	}
	if (hcan->Instance == CAN2) {
		flag_of_tx_1 = 1;
		lastActivityTimestamp1 = HAL_GetTick();
	}
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
	if (hcan->Instance == CAN1) {
		flag_of_tx = 1;
		lastActivityTimestamp = HAL_GetTick();
	}
	if (hcan->Instance == CAN2) {
		flag_of_tx_1 = 1;
		lastActivityTimestamp1 = HAL_GetTick();
	}
}

