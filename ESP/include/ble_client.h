#ifndef BLE_CLIENT_H
#define BLE_CLIENT_H

#include <stdint.h>
#include <stdbool.h>

typedef void (*ble_client_on_ready_cb_t)(void);

// Initialize BLE and begin scanning/connecting.
// target_name: advertised name of STM peripheral (e.g., "STM_ARM")
void ble_client_init(const char *target_name, ble_client_on_ready_cb_t on_ready);

// True when connected and RX characteristic handle discovered
bool ble_client_ready(void);

// Send bytes to STM RX characteristic (Write No Response if possible)
bool ble_client_send(const uint8_t *data, uint16_t len);

// Send text to a connected phone BLE UART terminal, such as Adafruit Bluefruit.
bool ble_uart_send_text(const char *text);

// True after the phone connects and opens/enables the UART terminal.
bool ble_uart_ready(void);

#endif
