#include "main.h"
#include "usart.h"
#include "i2c.h"
#include <math.h>
#include <stdio.h>
#include "cmsis_os.h"

/* UART-optical flow */
extern DMA_HandleTypeDef hdma_usart1_rx;
void OnNewMotionFrame();
void ProcessByte(uint8_t b);
void ProcessDMAData(void);
#define FRAME_MAX 18
#define DMA_BUFFER_SIZE 256
typedef struct {
	float x;      // см
	float y;      // см
	float phi;    // рад
	float vx;     // см/с
	float vy;     // см/с
	float omega;  // рад/с
} odom_t;
uint32_t distance_mm = 0;
int32_t flow_dx = 0;
int32_t flow_dy = 0;
uint8_t brightness = 0;
uint32_t last_time_us = 0;
static odom_t odom = { 0 };
static uint8_t dma_rx_buffer[DMA_BUFFER_SIZE];
static uint32_t dma_last_pos = 0;
static uint8_t frame[FRAME_MAX];
static uint8_t idx = 0;
static uint8_t expected_len = 0;
static float FLOW_K = 0.65f;

/* I2C1-gyroscope */
#define GYRO_STATIC_THRESH   0.1f
#define ACC_STATIC_THRESH    0.1f
#define STATIC_COUNT_LIMIT  30

extern I2C_HandleTypeDef hi2c1;
#define RAD_TO_DEG 57.2957795f

#define MPU6050_RESET     0x80
#define MPU6050_ADDR 0x68<<1
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_INT_PIN_CFG 0x37

int16_t gyro_x, gyro_y, gyro_z;
float gyro_offset_x = 76.514;
float gyro_offset_y = 242.478;
float gyro_offset_z = 168.709;

int16_t accel_x, accel_y, accel_z;
float accel_offset_x = -344.464f;
float accel_offset_y = -484.092f;
float accel_offset_z = 209.67969f;

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

#define BETA 0.01f
#define DT   0.01f
#define DEG2RAD 0.01745329251f
#define RAD2DEG 57.2957795f

#define LPF_ALPHA 0.2f

static float gyro_z_bias = 0.0f;
#define GYRO_BIAS_ALPHA  0.002f

static float yaw_frozen = 0.0f;
static uint16_t staticCounter = 0;
static uint8_t isStaticNow = 0;

#define MAX_TILT_ANGLE 30.0f
#define MIN_HEIGHT_MM 30

static float ax_f, ay_f, az_f;

// Anomaly deviation correction
#define MAX_PLAUSIBLE_VELOCITY_CM_S 150.0f  // Максимальная реалистичная скорость (1.5 м/с)
#define MAX_PLAUSIBLE_DISPLACEMENT_CM 15.0f  // Макс. смещение за один шаг (10 см)
#define OUTLIER_FILTER_ALPHA 0.2f

uint8_t IsTiltAcceptable(float pitch_deg, float roll_deg) {
	return (fabsf(pitch_deg) < MAX_TILT_ANGLE
			&& fabsf(roll_deg) < MAX_TILT_ANGLE);
}

void TransformFlowToWorld(int32_t flow_dx, int32_t flow_dy, float distance_m,
		float yaw_rad, float pitch_rad, float roll_rad, float *dx_world,
		float *dy_world) {
	float dx_sensor = (distance_m / FLOW_K) * (float) (flow_dx*2);
	float dy_sensor = (distance_m / FLOW_K) * (float) flow_dy;

	float cos_pitch = cosf(pitch_rad);
	float sin_pitch = sinf(pitch_rad);

	float cos_roll = cosf(roll_rad);
	float sin_roll = sinf(roll_rad);

	float cos_yaw = cosf(yaw_rad);
	float sin_yaw = sinf(yaw_rad);

	float R11 = cos_yaw * cos_pitch;
	float R12 = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;

	float R21 = sin_yaw * cos_pitch;
	float R22 = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;

	*dx_world = R11 * dx_sensor + R21 * dy_sensor;
	*dy_world = R12 * dx_sensor + R22 * dy_sensor;

}
void LowPassFilter(float *x_f, float x) {
	*x_f = *x_f + LPF_ALPHA * (x - *x_f);
}
void MPU6050_Write(uint8_t reg, uint8_t data) {
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, 1, &data, 1, 100);
}
void MPU6050_ReadGyro(void) {
	uint8_t buf[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 1, buf, 6, 100);
	gyro_x = (int16_t) (buf[0] << 8 | buf[1]);
	gyro_y = (int16_t) (buf[2] << 8 | buf[3]);
	gyro_z = (int16_t) (buf[4] << 8 | buf[5]);
}
void MPU6050_ReadAccel(void) {
	uint8_t buf[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,
	MPU6050_ACCEL_XOUT_H, 1, buf, 6, 100);
	accel_x = (int16_t) (buf[0] << 8 | buf[1]);
	accel_y = (int16_t) (buf[2] << 8 | buf[3]);
	accel_z = (int16_t) (buf[4] << 8 | buf[5]);
}

void MPU6050_Init(void) {
	MPU6050_Write(MPU6050_PWR_MGMT_1, 0x00);
	HAL_Delay(10);
	MPU6050_Write(MPU6050_GYRO_CONFIG, 0x00);
	MPU6050_Write(MPU6050_ACCEL_CONFIG, 0x00);
	HAL_Delay(10);
	MPU6050_Write(MPU6050_INT_PIN_CFG, 0x02);
}
void ReadSensors(float *ax, float *ay, float *az, float *gx, float *gy,
		float *gz) {
	MPU6050_ReadAccel();
	MPU6050_ReadGyro();
	*ax = (accel_x - accel_offset_x) / 16384.0f;
	*ay = (accel_y - accel_offset_y) / 16384.0f;
	*az = (accel_z - accel_offset_z) / 16384.0f;
	*gx = (gyro_x - gyro_offset_x) / 131.0f * DEG2RAD;
	*gy = (gyro_y - gyro_offset_y) / 131.0f * DEG2RAD;
	*gz = (gyro_z - gyro_offset_z) / 131.0f * DEG2RAD;
}

uint8_t DetectStatic(float ax, float ay, float az, float gx, float gy, float gz) {
	float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
	if (fabsf(acc_norm - 1.0f) < ACC_STATIC_THRESH
			&& fabsf(gx) < GYRO_STATIC_THRESH && fabsf(gy) < GYRO_STATIC_THRESH
			&& fabsf(gz) < GYRO_STATIC_THRESH) {
		if (staticCounter < STATIC_COUNT_LIMIT)
			staticCounter++;
	} else {
		staticCounter = 0;
	}
	isStaticNow = (staticCounter >= STATIC_COUNT_LIMIT);
	return isStaticNow;
}
void MadgwickUpdate(float ax, float ay, float az, float gx, float gy, float gz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0 = 2.0f * q0;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	recipNorm = sqrtf(ax * ax + ay * ay + az * az);
	if (recipNorm == 0.0f)
		return;
	recipNorm = 1.0f / recipNorm;
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;
	s0 = -_2q2 * (2 * (q1 * q3 - q0 * q2) - ax)
			+ _2q1 * (2 * (q0 * q1 + q2 * q3) - ay);

	s1 = _2q3 * (2 * (q1 * q3 - q0 * q2) - ax)
			+ _2q0 * (2 * (q0 * q1 + q2 * q3) - ay);

	s2 = -_2q0 * (2 * (q1 * q3 - q0 * q2) - ax)
			+ _2q3 * (2 * (q0 * q1 + q2 * q3) - ay);

	s3 = _2q1 * (2 * (q1 * q3 - q0 * q2) - ax)
			+ _2q2 * (2 * (q0 * q1 + q2 * q3) - ay);
	recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
	s0 *= recipNorm;
	s1 *= recipNorm;
	s2 *= recipNorm;
	s3 *= recipNorm;

	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - BETA * s0;
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - BETA * s1;
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - BETA * s2;
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - BETA * s3;

	q0 += qDot1 * DT;
	q1 += qDot2 * DT;
	q2 += qDot3 * DT;
	q3 += qDot4 * DT;

	recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}
void ComputeEulerAngles(void) {
	roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
			1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;
	pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * RAD2DEG;
	yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
			1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;
}
void StartOdometryTask(void *argument) {
	float ax, ay, az;
	float gx, gy, gz;
	MPU6050_Init();
	HAL_UART_Receive_DMA(&huart1, dma_rx_buffer, DMA_BUFFER_SIZE);
	for (;;) {
		ReadSensors(&ax, &ay, &az, &gx, &gy, &gz);
		LowPassFilter(&ax_f, ax);
		LowPassFilter(&ay_f, ay);
		LowPassFilter(&az_f, az);
		DetectStatic(ax_f, ay_f, az_f, gx, gy, gz);
		if (isStaticNow) {
			gyro_z_bias += GYRO_BIAS_ALPHA * (gz - gyro_z_bias);
		}
		gz -= gyro_z_bias;
		MadgwickUpdate(ax_f, ay_f, az_f, gx, gy, gz);
		ComputeEulerAngles();
		if (isStaticNow)
			yaw = yaw_frozen;
		else
			yaw_frozen = yaw;
		ProcessDMAData();
		osDelay(10);
	}
}
void ParseFrame(uint8_t *p, uint8_t len) {
	if (len < 14)
		return;
	if (!(p[0] == 0x24 && p[1] == 0x58 && p[2] == 0x3C && p[3] == 0x00))
		return;
	uint8_t type = p[4];
	if (type == 0x01 && len == 14) {
		distance_mm = (int32_t) (((uint32_t) p[9]) | (((uint32_t) p[10]) << 8)
				| (((uint32_t) p[11]) << 16) | (((uint32_t) p[12]) << 24));
		return;
	}
	if (type == 0x02 && len == 18) {
		brightness = p[8];
		flow_dy = (int32_t) (((uint32_t) p[9]) | (((uint32_t) p[10]) << 8)
				| (((uint32_t) p[11]) << 16) | (((uint32_t) p[12]) << 24));
		flow_dx = (int32_t) (((uint32_t) p[13]) | (((uint32_t) p[14]) << 8)
				| (((uint32_t) p[15]) << 16) | (((uint32_t) p[16]) << 24));
		OnNewMotionFrame();
		return;
	}
}

void ProcessDMAData(void) {
	uint32_t current_pos = DMA_BUFFER_SIZE
			- __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
	if (current_pos < dma_last_pos) {
		for (uint32_t i = dma_last_pos; i < DMA_BUFFER_SIZE; i++) {
			ProcessByte(dma_rx_buffer[i]);
		}
		dma_last_pos = 0;
	}
	for (uint32_t i = dma_last_pos; i < current_pos; i++) {
		ProcessByte(dma_rx_buffer[i]);
	}
	dma_last_pos = current_pos;
}

void ProcessByte(uint8_t b) {
	if (idx == 0) {
		if (b == 0x24) {
			frame[idx++] = b;
		}
		return;
	}
	frame[idx++] = b;
	if (idx == 5) {
		switch (frame[4]) {
		case 0x01:
			expected_len = 14;
			break;
		case 0x02:
			expected_len = 18;
			break;
		default:
			idx = 0;
			return;
		}
	}
	if (expected_len != 0 && idx == expected_len) {
		ParseFrame(frame, expected_len);
		idx = 0;
		expected_len = 0;
	}
}

static float lowpass(float prev, float cur, float alpha) {
	return prev + alpha * (cur - prev);
}

void Odom_Update(int32_t flow_dx, int32_t flow_dy, uint32_t distance_mm,
		float dt, float yaw_deg, float pitch_deg, float roll_deg) {
	static uint32_t outlier_count = 0;
	static const uint32_t MAX_OUTLIERS_BEFORE_RESET = 10;

	if (distance_mm < MIN_HEIGHT_MM) {
		odom.vx = 0.0f;
		odom.vy = 0.0f;
		outlier_count = 0;
		return;
	}

	if (!IsTiltAcceptable(pitch_deg, roll_deg)) {
		odom.vx = -odom.vx * 0.5f;
		odom.vy = odom.vy * 0.5f;
		outlier_count = 0;
		return;
	}

	float yaw_rad = yaw_deg * DEG2RAD;
	float pitch_rad = pitch_deg * DEG2RAD;
	float roll_rad = roll_deg * DEG2RAD;

	float dx_world = 0, dy_world = 0;
	TransformFlowToWorld(flow_dx, flow_dy, distance_mm / 1000.0f, yaw_rad,
			pitch_rad, roll_rad, &dx_world, &dy_world);

	// Проверка на аномально большие смещения
	float displacement_magnitude = sqrtf(
			dx_world * dx_world + dy_world * dy_world);

	if (displacement_magnitude > MAX_PLAUSIBLE_DISPLACEMENT_CM) {
		// Слишком большое смещение - вероятно шум
		outlier_count++;

		if (outlier_count > MAX_OUTLIERS_BEFORE_RESET) {
			// Слишком много выбросов подряд - сбрасываем скорость
			odom.vx = 0.0f;
			odom.vy = 0.0f;
		} else {
			// Частично учитываем, но с большим сглаживанием
			dx_world *= OUTLIER_FILTER_ALPHA;
			dy_world *= OUTLIER_FILTER_ALPHA;
		}
		return;
	} else {
		outlier_count = 0;  // Сбрасываем счетчик при нормальных значениях
	}

	const float deadzone = 0.02f;
	if (fabsf(dx_world) < deadzone)
		dx_world = 0.0f;
	if (fabsf(dy_world) < deadzone)
		dy_world = 0.0f;

	if (dx_world == 0.0f && dy_world == 0.0f) {
		odom.vx = 0.0f;
		odom.vy = 0.0f;
		return;
	}

	odom.x += dx_world;
	odom.y += dy_world;

	float vx_raw = dx_world / dt;
	float vy_raw = dy_world / dt;

	// Дополнительная проверка скорости
	float velocity_magnitude = sqrtf(vx_raw * vx_raw + vy_raw * vy_raw);
	if (velocity_magnitude > MAX_PLAUSIBLE_VELOCITY_CM_S) {
		// Скорость нереалистично высокая - игнорируем
		return;
	}

	const float alpha = 0.2f;
	odom.vx = lowpass(odom.vx, vx_raw, alpha);
	odom.vy = lowpass(odom.vy, vy_raw, alpha);

	odom.phi = yaw_rad;
	static float last_yaw_rad = 0;
	if (dt > 0) {
		float delta_yaw = yaw_rad - last_yaw_rad;
		// Нормализация разницы углов (-π до π)
		while (delta_yaw > M_PI)
			delta_yaw -= 2 * M_PI;
		while (delta_yaw < -M_PI)
			delta_yaw += 2 * M_PI;
		odom.omega = delta_yaw / dt;
	}
	last_yaw_rad = yaw_rad;
}
void OnNewMotionFrame() {
	uint32_t now = HAL_GetTick() * 1000;
	float dt = (now - last_time_us) / 1e6f;
	if (dt <= 0)
		dt = 0.01f;
	last_time_us = now;
	Odom_Update(flow_dx, flow_dy, distance_mm, dt, yaw, pitch, roll);
}
