#include <rmw_microxrcedds_c/config.h>
#include "main.h"
#include "cmsis_os.h"
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "uxr/client/transport.h"
#include "uxr/client/profile/transport/custom/custom_transport.h"
#define W5500_SPI_TIMEOUT 100
#define UDP_SOCKET 5

/* -----------------------------
 W5500: HAL callbacks
 ----------------------------- */
void W5500_Select(void) {
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
}
void W5500_Unselect(void) {
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
}
void W5500_ReadBuff(uint8_t *buff, uint16_t len) {
	HAL_SPI_Receive(&hspi1, buff, len, W5500_SPI_TIMEOUT);
}
void W5500_WriteBuff(uint8_t *buff, uint16_t len) {
	HAL_SPI_Transmit(&hspi1, buff, len, W5500_SPI_TIMEOUT);
}
uint8_t W5500_ReadByte(void) {
	uint8_t byte;
	W5500_ReadBuff(&byte, sizeof(byte));
	return byte;
}
void W5500_WriteByte(uint8_t byte) {
	W5500_WriteBuff(&byte, sizeof(byte));
}

/* -----------------------------
 Network config
 ----------------------------- */
wiz_NetInfo gWIZNETINFO = { .mac = { 0x00, 0x08, 0xDC, 0xAB, 0xCD, 0xEF }, .ip =
		{ 192, 168, 10, 3 }, // IP MCU (W5500)
		.sn = { 255, 255, 255, 0 }, .dns = { 0, 0, 0, 0 }, .dhcp =
				NETINFO_STATIC };

/* Агент (Ubuntu) — фиксированный адрес/порт */
static uint8_t agent_ip[4] = { 192, 168, 10, 2 };
static uint16_t agent_port = 33333;

/* опционально — для диагностики (не используется для изменения agent) */
static uint8_t last_peer_ip[4] = { 0 };
static uint16_t last_peer_port = 0;

/* -----------------------------
 W5500 init
 ----------------------------- */
void W5500_Init(void) {
	// Reset W5500
	HAL_GPIO_WritePin(SPI1_RESET_GPIO_Port, SPI1_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(SPI1_RESET_GPIO_Port, SPI1_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
	reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
	reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);

	uint8_t rx_tx_buff_sizes[] = { 0, 0, 0, 0, 0, 8, 0, 0 };
	wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

	// Set network info
	wizchip_setnetinfo(&gWIZNETINFO);
	ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);

	// Open UDP socket on dest_Port (MCU local port). Non-blocking flag = 1 (we poll in read).
	socket(UDP_SOCKET, Sn_MR_UDP, agent_port, SF_IO_NONBLOCK);
}

/* -----------------------------
 Transport callbacks required by micro-ROS
 ----------------------------- */
bool w5500_transport_open(struct uxrCustomTransport *transport) {
	(void) transport;
	return true;
}

bool w5500_transport_close(struct uxrCustomTransport *transport) {
	(void) transport;
	return true;
}
size_t w5500_transport_write(struct uxrCustomTransport *transport,
		const uint8_t *buf, size_t len, uint8_t *err) {
	(void) transport;
	int32_t sent = sendto(UDP_SOCKET, (uint8_t*) buf, (uint16_t) len, agent_ip,
			agent_port);
	if (sent >= 0) {
		if (err)
			*err = 0;
		return (size_t) sent;
	} else {
		if (err)
			*err = 1;
		return 0;
	}
}

size_t w5500_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
		size_t len, int timeout_ms, uint8_t *err) {
	(void) transport;
	if (err)
		*err = 0;

	TickType_t start = xTaskGetTickCount();
	TickType_t timeout_ticks =
			(timeout_ms < 0) ?
					portMAX_DELAY : pdMS_TO_TICKS((uint32_t )timeout_ms);

	for (;;) {
		uint16_t available = getSn_RX_RSR(UDP_SOCKET);
		if (available > 0) {
			uint16_t to_read = (available > len) ? (uint16_t) len : available;

			uint8_t remote_ip[4];
			uint16_t remote_port;
			int32_t ret = recvfrom(UDP_SOCKET, buf, to_read, remote_ip,
					&remote_port);
			if (ret <= 0) {
				if (err)
					*err = 1;
				return 0;
			}

			// Сохраняем peer лишь для диагностических целей (НЕ используем для ответа)
			memcpy(last_peer_ip, remote_ip, 4);
			last_peer_port = remote_port;

			return (size_t) ret;
		}
	}
}
