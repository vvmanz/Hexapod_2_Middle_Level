#include "main.h"

int32_t set_angle1_1, set_angle1_2, set_angle2_1, set_angle2_2, set_angle3_1,
		set_angle3_2;

int32_t setpoint_user_1_1, setpoint_user_1_2, setpoint_user_2_1,
		setpoint_user_2_2, setpoint_user_3_1, setpoint_user_3_2;

int32_t prev_user_1_1, prev_user_1_2, prev_user_2_1, prev_user_2_2,
		prev_user_3_1, prev_user_3_2;

int32_t angle1_1, angle1_2, angle2_1, angle2_2, angle3_1, angle3_2, speed1_1,
		speed1_2, speed2_1, speed2_2, speed3_1, speed3_2, current1_1,
		current1_2, current2_1, current2_2, current3_1, current3_2;

int32_t kp_angle_1_1, kp_angle_1_2, kp_angle_2_1, kp_angle_2_2, kp_angle_3_1,
		kp_angle_3_2;
int32_t ki_angle_1_1, ki_angle_1_2, ki_angle_2_1, ki_angle_2_2, ki_angle_3_1,
		ki_angle_3_2;
int32_t kd_angle_1_1, kd_angle_1_2, kd_angle_2_1, kd_angle_2_2, kd_angle_3_1,
		kd_angle_3_2;
int32_t kp_speed_1_1, kp_speed_1_2, kp_speed_2_1, kp_speed_2_2, kp_speed_3_1,
		kp_speed_3_2;
int32_t ki_speed_1_1, ki_speed_1_2, ki_speed_2_1, ki_speed_2_2, ki_speed_3_1,
		ki_speed_3_2;
int32_t kp_curr_1_1, kp_curr_1_2, kp_curr_2_1, kp_curr_2_2, kp_curr_3_1,
		kp_curr_3_2;
int32_t ki_curr_1_1, ki_curr_1_2, ki_curr_2_1, ki_curr_2_2, ki_curr_3_1,
		ki_curr_3_2;

int16_t obstacle1_1, obstacle1_2, obstacle2_1, obstacle2_2, obstacle3_1,
		obstacle3_2;
uint8_t speed_limit1_1, speed_limit1_2, speed_limit2_1, speed_limit2_2,
		speed_limit3_1, speed_limit3_2;
int8_t phase1_1, phase1_2, phase2_1, phase2_2, phase3_1, phase3_2;

int8_t set_zero;
int8_t system_type1_1, system_type1_2, system_type2_1, system_type2_2,
		system_type3_1, system_type3_2;

