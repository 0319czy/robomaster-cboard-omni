#ifndef RC_SERIAL_H
#define RC_SERIAL_H
#include <stdint.h>
/* DBUS 口 (PC11 = UART3_RX)。按 100 kbps 接收，先抓原始字节判断协议。 */
void RcSerial_Init(uint32_t baud, uint32_t parity); /* parity: 0=无 1=偶 2=奇 */
extern volatile uint32_t g_rc_serial_bytes, g_rc_serial_errors;
extern volatile uint8_t g_rc_raw[256];
void RcSerial_OnRxComplete(void);
void RcSerial_OnError(void);
extern volatile uint32_t g_rc_scan[16];
extern volatile uint32_t g_sbus[20];
extern volatile uint32_t g_sbus_range[34];
extern volatile uint32_t g_sbus_last_ms;
void RcSerial_ScanStep(uint32_t now);
#endif
