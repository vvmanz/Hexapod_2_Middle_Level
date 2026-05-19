#include "main.h"

int32_t ConvertBytesToInt32(uint8_t *data) {
	return (int32_t) ((data[0] << 24) | (data[1] << 16) | (data[2] << 8)
			| data[3]);
}

void BridgeDecode(int a) {
	if (a == 1) {
		angle1_1 = ConvertBytesToInt32(RxAngles1);
		angle1_2 = ConvertBytesToInt32(&RxAngles1[4]);

		speed1_1 = ConvertBytesToInt32(RxSpeeds1);
		speed1_2 = ConvertBytesToInt32(&RxSpeeds1[4]);

		current1_1 = ConvertBytesToInt32(RxCurrents1);
		current1_2 = ConvertBytesToInt32(&RxCurrents1[4]);
	} else if (a == 2) {
		angle2_1 = ConvertBytesToInt32(RxAngles2);
		angle2_2 = ConvertBytesToInt32(&RxAngles2[4]);

		speed2_1 = ConvertBytesToInt32(RxSpeeds2);
		speed2_2 = ConvertBytesToInt32(&RxSpeeds2[4]);

		current2_1 = ConvertBytesToInt32(RxCurrents2);
		current2_2 = ConvertBytesToInt32(&RxCurrents2[4]);
	} else if (a == 3) {
		angle3_1 = ConvertBytesToInt32(RxAngles3);
		angle3_2 = ConvertBytesToInt32(&RxAngles3[4]);

		speed3_1 = ConvertBytesToInt32(RxSpeeds3);
		speed3_2 = ConvertBytesToInt32(&RxSpeeds3[4]);

		current3_1 = ConvertBytesToInt32(RxCurrents3);
		current3_2 = ConvertBytesToInt32(&RxCurrents3[4]);
	}
}

void Prepare_set_koeffs() {
	// kp ang
	TxDataCoeffs1_1[0] = (uint8_t) ((kp_angle_1_1 >> 0) & 0xFF);
	TxDataCoeffs1_1[1] = (uint8_t) ((kp_angle_1_1 >> 8) & 0xFF);
	TxDataCoeffs1_1[2] = (uint8_t) ((kp_angle_1_1 >> 16) & 0xFF);
	TxDataCoeffs1_1[3] = (uint8_t) ((kp_angle_1_1 >> 24) & 0xFF);

	TxDataCoeffs1_1[4] = (uint8_t) ((kp_angle_1_2 >> 0) & 0xFF);
	TxDataCoeffs1_1[5] = (uint8_t) ((kp_angle_1_2 >> 8) & 0xFF);
	TxDataCoeffs1_1[6] = (uint8_t) ((kp_angle_1_2 >> 16) & 0xFF);
	TxDataCoeffs1_1[7] = (uint8_t) ((kp_angle_1_2 >> 24) & 0xFF);

	TxDataCoeffs2_1[0] = (uint8_t) ((kp_angle_2_1 >> 0) & 0xFF);
	TxDataCoeffs2_1[1] = (uint8_t) ((kp_angle_2_1 >> 8) & 0xFF);
	TxDataCoeffs2_1[2] = (uint8_t) ((kp_angle_2_1 >> 16) & 0xFF);
	TxDataCoeffs2_1[3] = (uint8_t) ((kp_angle_2_1 >> 24) & 0xFF);

	TxDataCoeffs2_1[4] = (uint8_t) ((kp_angle_2_2 >> 0) & 0xFF);
	TxDataCoeffs2_1[5] = (uint8_t) ((kp_angle_2_2 >> 8) & 0xFF);
	TxDataCoeffs2_1[6] = (uint8_t) ((kp_angle_2_2 >> 16) & 0xFF);
	TxDataCoeffs2_1[7] = (uint8_t) ((kp_angle_2_2 >> 24) & 0xFF);

	TxDataCoeffs3_1[0] = (uint8_t) ((kp_angle_3_1 >> 0) & 0xFF);
	TxDataCoeffs3_1[1] = (uint8_t) ((kp_angle_3_1 >> 8) & 0xFF);
	TxDataCoeffs3_1[2] = (uint8_t) ((kp_angle_3_1 >> 16) & 0xFF);
	TxDataCoeffs3_1[3] = (uint8_t) ((kp_angle_3_1 >> 24) & 0xFF);

	TxDataCoeffs3_1[4] = (uint8_t) ((kp_angle_3_2 >> 0) & 0xFF);
	TxDataCoeffs3_1[5] = (uint8_t) ((kp_angle_3_2 >> 8) & 0xFF);
	TxDataCoeffs3_1[6] = (uint8_t) ((kp_angle_3_2 >> 16) & 0xFF);
	TxDataCoeffs3_1[7] = (uint8_t) ((kp_angle_3_2 >> 24) & 0xFF);

	// ki ang
	TxDataCoeffs1_2[0] = (uint8_t) ((ki_angle_1_1 >> 0) & 0xFF);
	TxDataCoeffs1_2[1] = (uint8_t) ((ki_angle_1_1 >> 8) & 0xFF);
	TxDataCoeffs1_2[2] = (uint8_t) ((ki_angle_1_1 >> 16) & 0xFF);
	TxDataCoeffs1_2[3] = (uint8_t) ((ki_angle_1_1 >> 24) & 0xFF);

	TxDataCoeffs1_2[4] = (uint8_t) ((ki_angle_1_2 >> 0) & 0xFF);
	TxDataCoeffs1_2[5] = (uint8_t) ((ki_angle_1_2 >> 8) & 0xFF);
	TxDataCoeffs1_2[6] = (uint8_t) ((ki_angle_1_2 >> 16) & 0xFF);
	TxDataCoeffs1_2[7] = (uint8_t) ((ki_angle_1_2 >> 24) & 0xFF);

	TxDataCoeffs2_2[0] = (uint8_t) ((ki_angle_2_1 >> 0) & 0xFF);
	TxDataCoeffs2_2[1] = (uint8_t) ((ki_angle_2_1 >> 8) & 0xFF);
	TxDataCoeffs2_2[2] = (uint8_t) ((ki_angle_2_1 >> 16) & 0xFF);
	TxDataCoeffs2_2[3] = (uint8_t) ((ki_angle_2_1 >> 24) & 0xFF);

	TxDataCoeffs2_2[4] = (uint8_t) ((ki_angle_2_2 >> 0) & 0xFF);
	TxDataCoeffs2_2[5] = (uint8_t) ((ki_angle_2_2 >> 8) & 0xFF);
	TxDataCoeffs2_2[6] = (uint8_t) ((ki_angle_2_2 >> 16) & 0xFF);
	TxDataCoeffs2_2[7] = (uint8_t) ((ki_angle_2_2 >> 24) & 0xFF);

	TxDataCoeffs3_2[0] = (uint8_t) ((ki_angle_3_1 >> 0) & 0xFF);
	TxDataCoeffs3_2[1] = (uint8_t) ((ki_angle_3_1 >> 8) & 0xFF);
	TxDataCoeffs3_2[2] = (uint8_t) ((ki_angle_3_1 >> 16) & 0xFF);
	TxDataCoeffs3_2[3] = (uint8_t) ((ki_angle_3_1 >> 24) & 0xFF);

	TxDataCoeffs3_2[4] = (uint8_t) ((ki_angle_3_2 >> 0) & 0xFF);
	TxDataCoeffs3_2[5] = (uint8_t) ((ki_angle_3_2 >> 8) & 0xFF);
	TxDataCoeffs3_2[6] = (uint8_t) ((ki_angle_3_2 >> 16) & 0xFF);
	TxDataCoeffs3_2[7] = (uint8_t) ((ki_angle_3_2 >> 24) & 0xFF);

	// kd ang
	TxDataCoeffs1_3[0] = (uint8_t) ((kd_angle_1_1 >> 0) & 0xFF);
	TxDataCoeffs1_3[1] = (uint8_t) ((kd_angle_1_1 >> 8) & 0xFF);
	TxDataCoeffs1_3[2] = (uint8_t) ((kd_angle_1_1 >> 16) & 0xFF);
	TxDataCoeffs1_3[3] = (uint8_t) ((kd_angle_1_1 >> 24) & 0xFF);

	TxDataCoeffs1_3[4] = (uint8_t) ((kd_angle_1_2 >> 0) & 0xFF);
	TxDataCoeffs1_3[5] = (uint8_t) ((kd_angle_1_2 >> 8) & 0xFF);
	TxDataCoeffs1_3[6] = (uint8_t) ((kd_angle_1_2 >> 16) & 0xFF);
	TxDataCoeffs1_3[7] = (uint8_t) ((kd_angle_1_2 >> 24) & 0xFF);

	TxDataCoeffs2_3[0] = (uint8_t) ((kd_angle_2_1 >> 0) & 0xFF);
	TxDataCoeffs2_3[1] = (uint8_t) ((kd_angle_2_1 >> 8) & 0xFF);
	TxDataCoeffs2_3[2] = (uint8_t) ((kd_angle_2_1 >> 16) & 0xFF);
	TxDataCoeffs2_3[3] = (uint8_t) ((kd_angle_2_1 >> 24) & 0xFF);

	TxDataCoeffs2_3[4] = (uint8_t) ((kd_angle_2_2 >> 0) & 0xFF);
	TxDataCoeffs2_3[5] = (uint8_t) ((kd_angle_2_2 >> 8) & 0xFF);
	TxDataCoeffs2_3[6] = (uint8_t) ((kd_angle_2_2 >> 16) & 0xFF);
	TxDataCoeffs2_3[7] = (uint8_t) ((kd_angle_2_2 >> 24) & 0xFF);

	TxDataCoeffs3_3[0] = (uint8_t) ((kd_angle_3_1 >> 0) & 0xFF);
	TxDataCoeffs3_3[1] = (uint8_t) ((kd_angle_3_1 >> 8) & 0xFF);
	TxDataCoeffs3_3[2] = (uint8_t) ((kd_angle_3_1 >> 16) & 0xFF);
	TxDataCoeffs3_3[3] = (uint8_t) ((kd_angle_3_1 >> 24) & 0xFF);

	TxDataCoeffs3_3[4] = (uint8_t) ((kd_angle_3_2 >> 0) & 0xFF);
	TxDataCoeffs3_3[5] = (uint8_t) ((kd_angle_3_2 >> 8) & 0xFF);
	TxDataCoeffs3_3[6] = (uint8_t) ((kd_angle_3_2 >> 16) & 0xFF);
	TxDataCoeffs3_3[7] = (uint8_t) ((kd_angle_3_2 >> 24) & 0xFF);

	// kp speed
	TxDataCoeffs1_4[0] = (uint8_t) ((kp_speed_1_1 >> 0) & 0xFF);
	TxDataCoeffs1_4[1] = (uint8_t) ((kp_speed_1_1 >> 8) & 0xFF);
	TxDataCoeffs1_4[2] = (uint8_t) ((kp_speed_1_1 >> 16) & 0xFF);
	TxDataCoeffs1_4[3] = (uint8_t) ((kp_speed_1_1 >> 24) & 0xFF);

	TxDataCoeffs1_4[4] = (uint8_t) ((kp_speed_1_2 >> 0) & 0xFF);
	TxDataCoeffs1_4[5] = (uint8_t) ((kp_speed_1_2 >> 8) & 0xFF);
	TxDataCoeffs1_4[6] = (uint8_t) ((kp_speed_1_2 >> 16) & 0xFF);
	TxDataCoeffs1_4[7] = (uint8_t) ((kp_speed_1_2 >> 24) & 0xFF);

	TxDataCoeffs2_4[0] = (uint8_t) ((kp_speed_2_1 >> 0) & 0xFF);
	TxDataCoeffs2_4[1] = (uint8_t) ((kp_speed_2_1 >> 8) & 0xFF);
	TxDataCoeffs2_4[2] = (uint8_t) ((kp_speed_2_1 >> 16) & 0xFF);
	TxDataCoeffs2_4[3] = (uint8_t) ((kp_speed_2_1 >> 24) & 0xFF);

	TxDataCoeffs2_4[4] = (uint8_t) ((kp_speed_2_2 >> 0) & 0xFF);
	TxDataCoeffs2_4[5] = (uint8_t) ((kp_speed_2_2 >> 8) & 0xFF);
	TxDataCoeffs2_4[6] = (uint8_t) ((kp_speed_2_2 >> 16) & 0xFF);
	TxDataCoeffs2_4[7] = (uint8_t) ((kp_speed_2_2 >> 24) & 0xFF);

	TxDataCoeffs3_4[0] = (uint8_t) ((kp_speed_3_1 >> 0) & 0xFF);
	TxDataCoeffs3_4[1] = (uint8_t) ((kp_speed_3_1 >> 8) & 0xFF);
	TxDataCoeffs3_4[2] = (uint8_t) ((kp_speed_3_1 >> 16) & 0xFF);
	TxDataCoeffs3_4[3] = (uint8_t) ((kp_speed_3_1 >> 24) & 0xFF);

	TxDataCoeffs3_4[4] = (uint8_t) ((kp_speed_3_2 >> 0) & 0xFF);
	TxDataCoeffs3_4[5] = (uint8_t) ((kp_speed_3_2 >> 8) & 0xFF);
	TxDataCoeffs3_4[6] = (uint8_t) ((kp_speed_3_2 >> 16) & 0xFF);
	TxDataCoeffs3_4[7] = (uint8_t) ((kp_speed_3_2 >> 24) & 0xFF);

	// ki speed
	TxDataCoeffs1_5[0] = (uint8_t) ((ki_speed_1_1 >> 0) & 0xFF);
	TxDataCoeffs1_5[1] = (uint8_t) ((ki_speed_1_1 >> 8) & 0xFF);
	TxDataCoeffs1_5[2] = (uint8_t) ((ki_speed_1_1 >> 16) & 0xFF);
	TxDataCoeffs1_5[3] = (uint8_t) ((ki_speed_1_1 >> 24) & 0xFF);

	TxDataCoeffs1_5[4] = (uint8_t) ((ki_speed_1_2 >> 0) & 0xFF);
	TxDataCoeffs1_5[5] = (uint8_t) ((ki_speed_1_2 >> 8) & 0xFF);
	TxDataCoeffs1_5[6] = (uint8_t) ((ki_speed_1_2 >> 16) & 0xFF);
	TxDataCoeffs1_5[7] = (uint8_t) ((ki_speed_1_2 >> 24) & 0xFF);

	TxDataCoeffs2_5[0] = (uint8_t) ((ki_speed_2_1 >> 0) & 0xFF);
	TxDataCoeffs2_5[1] = (uint8_t) ((ki_speed_2_1 >> 8) & 0xFF);
	TxDataCoeffs2_5[2] = (uint8_t) ((ki_speed_2_1 >> 16) & 0xFF);
	TxDataCoeffs2_5[3] = (uint8_t) ((ki_speed_2_1 >> 24) & 0xFF);

	TxDataCoeffs2_5[4] = (uint8_t) ((ki_speed_2_2 >> 0) & 0xFF);
	TxDataCoeffs2_5[5] = (uint8_t) ((ki_speed_2_2 >> 8) & 0xFF);
	TxDataCoeffs2_5[6] = (uint8_t) ((ki_speed_2_2 >> 16) & 0xFF);
	TxDataCoeffs2_5[7] = (uint8_t) ((ki_speed_2_2 >> 24) & 0xFF);

	TxDataCoeffs3_5[0] = (uint8_t) ((ki_speed_3_1 >> 0) & 0xFF);
	TxDataCoeffs3_5[1] = (uint8_t) ((ki_speed_3_1 >> 8) & 0xFF);
	TxDataCoeffs3_5[2] = (uint8_t) ((ki_speed_3_1 >> 16) & 0xFF);
	TxDataCoeffs3_5[3] = (uint8_t) ((ki_speed_3_1 >> 24) & 0xFF);

	TxDataCoeffs3_5[4] = (uint8_t) ((ki_speed_3_2 >> 0) & 0xFF);
	TxDataCoeffs3_5[5] = (uint8_t) ((ki_speed_3_2 >> 8) & 0xFF);
	TxDataCoeffs3_5[6] = (uint8_t) ((ki_speed_3_2 >> 16) & 0xFF);
	TxDataCoeffs3_5[7] = (uint8_t) ((ki_speed_3_2 >> 24) & 0xFF);

	// kp curr
	TxDataCoeffs1_6[0] = (uint8_t) ((kp_curr_1_1 >> 0) & 0xFF);
	TxDataCoeffs1_6[1] = (uint8_t) ((kp_curr_1_1 >> 8) & 0xFF);
	TxDataCoeffs1_6[2] = (uint8_t) ((kp_curr_1_1 >> 16) & 0xFF);
	TxDataCoeffs1_6[3] = (uint8_t) ((kp_curr_1_1 >> 24) & 0xFF);

	TxDataCoeffs1_6[4] = (uint8_t) ((kp_curr_1_2 >> 0) & 0xFF);
	TxDataCoeffs1_6[5] = (uint8_t) ((kp_curr_1_2 >> 8) & 0xFF);
	TxDataCoeffs1_6[6] = (uint8_t) ((kp_curr_1_2 >> 16) & 0xFF);
	TxDataCoeffs1_6[7] = (uint8_t) ((kp_curr_1_2 >> 24) & 0xFF);

	TxDataCoeffs2_6[0] = (uint8_t) ((kp_curr_2_1 >> 0) & 0xFF);
	TxDataCoeffs2_6[1] = (uint8_t) ((kp_curr_2_1 >> 8) & 0xFF);
	TxDataCoeffs2_6[2] = (uint8_t) ((kp_curr_2_1 >> 16) & 0xFF);
	TxDataCoeffs2_6[3] = (uint8_t) ((kp_curr_2_1 >> 24) & 0xFF);

	TxDataCoeffs2_6[4] = (uint8_t) ((kp_curr_2_2 >> 0) & 0xFF);
	TxDataCoeffs2_6[5] = (uint8_t) ((kp_curr_2_2 >> 8) & 0xFF);
	TxDataCoeffs2_6[6] = (uint8_t) ((kp_curr_2_2 >> 16) & 0xFF);
	TxDataCoeffs2_6[7] = (uint8_t) ((kp_curr_2_2 >> 24) & 0xFF);

	TxDataCoeffs3_6[0] = (uint8_t) ((kp_curr_3_1 >> 0) & 0xFF);
	TxDataCoeffs3_6[1] = (uint8_t) ((kp_curr_3_1 >> 8) & 0xFF);
	TxDataCoeffs3_6[2] = (uint8_t) ((kp_curr_3_1 >> 16) & 0xFF);
	TxDataCoeffs3_6[3] = (uint8_t) ((kp_curr_3_1 >> 24) & 0xFF);

	TxDataCoeffs3_6[4] = (uint8_t) ((kp_curr_3_2 >> 0) & 0xFF);
	TxDataCoeffs3_6[5] = (uint8_t) ((kp_curr_3_2 >> 8) & 0xFF);
	TxDataCoeffs3_6[6] = (uint8_t) ((kp_curr_3_2 >> 16) & 0xFF);
	TxDataCoeffs3_6[7] = (uint8_t) ((kp_curr_3_2 >> 24) & 0xFF);

	// ki curr
	TxDataCoeffs1_7[0] = (uint8_t) ((ki_curr_1_1 >> 0) & 0xFF);
	TxDataCoeffs1_7[1] = (uint8_t) ((ki_curr_1_1 >> 8) & 0xFF);
	TxDataCoeffs1_7[2] = (uint8_t) ((ki_curr_1_1 >> 16) & 0xFF);
	TxDataCoeffs1_7[3] = (uint8_t) ((ki_curr_1_1 >> 24) & 0xFF);

	TxDataCoeffs1_7[4] = (uint8_t) ((ki_curr_1_2 >> 0) & 0xFF);
	TxDataCoeffs1_7[5] = (uint8_t) ((ki_curr_1_2 >> 8) & 0xFF);
	TxDataCoeffs1_7[6] = (uint8_t) ((ki_curr_1_2 >> 16) & 0xFF);
	TxDataCoeffs1_7[7] = (uint8_t) ((ki_curr_1_2 >> 24) & 0xFF);

	TxDataCoeffs2_7[0] = (uint8_t) ((ki_curr_2_1 >> 0) & 0xFF);
	TxDataCoeffs2_7[1] = (uint8_t) ((ki_curr_2_1 >> 8) & 0xFF);
	TxDataCoeffs2_7[2] = (uint8_t) ((ki_curr_2_1 >> 16) & 0xFF);
	TxDataCoeffs2_7[3] = (uint8_t) ((ki_curr_2_1 >> 24) & 0xFF);

	TxDataCoeffs2_7[4] = (uint8_t) ((ki_curr_2_2 >> 0) & 0xFF);
	TxDataCoeffs2_7[5] = (uint8_t) ((ki_curr_2_2 >> 8) & 0xFF);
	TxDataCoeffs2_7[6] = (uint8_t) ((ki_curr_2_2 >> 16) & 0xFF);
	TxDataCoeffs2_7[7] = (uint8_t) ((ki_curr_2_2 >> 24) & 0xFF);

	TxDataCoeffs3_7[0] = (uint8_t) ((ki_curr_3_1 >> 0) & 0xFF);
	TxDataCoeffs3_7[1] = (uint8_t) ((ki_curr_3_1 >> 8) & 0xFF);
	TxDataCoeffs3_7[2] = (uint8_t) ((ki_curr_3_1 >> 16) & 0xFF);
	TxDataCoeffs3_7[3] = (uint8_t) ((ki_curr_3_1 >> 24) & 0xFF);

	TxDataCoeffs3_7[4] = (uint8_t) ((ki_curr_3_2 >> 0) & 0xFF);
	TxDataCoeffs3_7[5] = (uint8_t) ((ki_curr_3_2 >> 8) & 0xFF);
	TxDataCoeffs3_7[6] = (uint8_t) ((ki_curr_3_2 >> 16) & 0xFF);
	TxDataCoeffs3_7[7] = (uint8_t) ((ki_curr_3_2 >> 24) & 0xFF);

}
void Prepare_set_angles() {
	TxDataAngles1[0] = (uint8_t) ((set_angle1_1 >> 0) & 0xFF);
	TxDataAngles1[1] = (uint8_t) ((set_angle1_1 >> 8) & 0xFF);
	TxDataAngles1[2] = (uint8_t) ((set_angle1_1 >> 16) & 0xFF);
	TxDataAngles1[3] = (uint8_t) ((set_angle1_1 >> 24) & 0xFF);

	TxDataAngles1[4] = (uint8_t) ((set_angle1_2 >> 0) & 0xFF);
	TxDataAngles1[5] = (uint8_t) ((set_angle1_2 >> 8) & 0xFF);
	TxDataAngles1[6] = (uint8_t) ((set_angle1_2 >> 16) & 0xFF);
	TxDataAngles1[7] = (uint8_t) ((set_angle1_2 >> 24) & 0xFF);

	TxDataAngles2[0] = (uint8_t) ((set_angle2_1 >> 0) & 0xFF);
	TxDataAngles2[1] = (uint8_t) ((set_angle2_1 >> 8) & 0xFF);
	TxDataAngles2[2] = (uint8_t) ((set_angle2_1 >> 16) & 0xFF);
	TxDataAngles2[3] = (uint8_t) ((set_angle2_1 >> 24) & 0xFF);

	TxDataAngles2[4] = (uint8_t) ((set_angle2_2 >> 0) & 0xFF);
	TxDataAngles2[5] = (uint8_t) ((set_angle2_2 >> 8) & 0xFF);
	TxDataAngles2[6] = (uint8_t) ((set_angle2_2 >> 16) & 0xFF);
	TxDataAngles2[7] = (uint8_t) ((set_angle2_2 >> 24) & 0xFF);

	TxDataAngles3[0] = (uint8_t) ((set_angle3_1 >> 0) & 0xFF);
	TxDataAngles3[1] = (uint8_t) ((set_angle3_1 >> 8) & 0xFF);
	TxDataAngles3[2] = (uint8_t) ((set_angle3_1 >> 16) & 0xFF);
	TxDataAngles3[3] = (uint8_t) ((set_angle3_1 >> 24) & 0xFF);

	TxDataAngles3[4] = (uint8_t) ((set_angle3_2 >> 0) & 0xFF);
	TxDataAngles3[5] = (uint8_t) ((set_angle3_2 >> 8) & 0xFF);
	TxDataAngles3[6] = (uint8_t) ((set_angle3_2 >> 16) & 0xFF);
	TxDataAngles3[7] = (uint8_t) ((set_angle3_2 >> 24) & 0xFF);

	TxDataOther1[0] = (uint8_t) (obstacle1_1 & 0xFF);
	TxDataOther1[1] = (uint8_t) (obstacle1_2 & 0xFF);
	{
		uint8_t b = 0;
		b |= ((uint8_t) phase1_1 & 0x01) << 0;             // bit0
		b |= ((uint8_t) phase1_2 & 0x01) << 1;             // bit1
		b |= ((uint8_t) set_zero & 0x01) << 2;          // bit2
		b |= ((uint8_t) system_type1_1 & 0x07) << 3;      // bits3-5
		b |= ((uint8_t) system_type1_2 & 0x03) << 6; // bits6-7 = lower 2 bits of sys2
		TxDataOther1[2] = b;
		TxDataOther1[3] = (uint8_t) (((uint8_t) system_type1_2 >> 2) & 0x01); // bit0 = upper bit of sys2
	}
	TxDataOther1[4] = (uint8_t) speed_limit1_1;
	TxDataOther1[5] = (uint8_t) speed_limit1_2;

	TxDataOther2[0] = (uint8_t) (obstacle2_1 & 0xFF);
	TxDataOther2[1] = (uint8_t) (obstacle2_2 & 0xFF);
	{
		uint8_t b = 0;
		b |= ((uint8_t) phase2_1 & 0x01) << 0;
		b |= ((uint8_t) phase2_2 & 0x01) << 1;
		b |= ((uint8_t) set_zero & 0x01) << 2;
		b |= ((uint8_t) system_type2_1 & 0x07) << 3;
		b |= ((uint8_t) system_type2_2 & 0x03) << 6;
		TxDataOther2[2] = b;
		TxDataOther2[3] = (uint8_t) (((uint8_t) system_type2_2 >> 2) & 0x01);
	}
	TxDataOther2[4] = (uint8_t) speed_limit2_1;
	TxDataOther2[5] = (uint8_t) speed_limit2_2;

	TxDataOther3[0] = (uint8_t) (obstacle3_1 & 0xFF);
	TxDataOther3[1] = (uint8_t) (obstacle3_2 & 0xFF);
	{
		uint8_t b = 0;
		b |= ((uint8_t) phase3_1 & 0x01) << 0;
		b |= ((uint8_t) phase3_2 & 0x01) << 1;
		b |= ((uint8_t) set_zero & 0x01) << 2;
		b |= ((uint8_t) system_type3_1 & 0x07) << 3;
		b |= ((uint8_t) system_type3_2 & 0x03) << 6;
		TxDataOther3[2] = b;
		TxDataOther3[3] = (uint8_t) (((uint8_t) system_type3_2 >> 2) & 0x01);
	}
	TxDataOther3[4] = (uint8_t) speed_limit3_1;
	TxDataOther3[5] = (uint8_t) speed_limit3_2;
}

