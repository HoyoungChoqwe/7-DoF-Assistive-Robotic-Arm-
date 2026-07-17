#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "joystick.h"
#include "ble_client.h"

// Packet format: AA 55 LEN ID PAYLOAD... CHK (sum of LEN+ID+payload)
#define PKT_SOF1 0xAA
#define PKT_SOF2 0x55
#define MSG_ID_JOYSTICK 0x10

#define STM_ADV_NAME "STM_ARM"   // must match STM advertising name

static uint8_t checksum_calc(uint8_t len, uint8_t id, const uint8_t *payload)
{
    uint16_t sum = (uint16_t)len + (uint16_t)id;
    for (uint8_t i = 0; i < len; i++) sum += payload[i];
    return (uint8_t)(sum & 0xFF);
}

static int build_packet(uint8_t id, const uint8_t *payload, uint8_t len, uint8_t *out, int out_max)
{
    const int total = 2 + 1 + 1 + (int)len + 1;
    if (out_max < total) return -1;

    out[0] = PKT_SOF1;
    out[1] = PKT_SOF2;
    out[2] = len;
    out[3] = id;
    if (len && payload) memcpy(&out[4], payload, len);
    out[4 + len] = checksum_calc(len, id, payload);
    return total;
}

static void ble_ready_cb(void)
{
    printf("BLE READY: connected + RX characteristic found\n");
    ble_uart_send_text("BLE READY: connected to STM\r\n");
}

// Sends joystick changes to the phone UART and, when connected, to the STM.
static void sender_task(void *arg)
{
    (void)arg;

    JoyDirection last = (JoyDirection)0xFF;
    uint8_t pkt[16];

    while (1) {
        JoyDirection d = Joystick_Get();
        if (d != last) {
            printf("Joystick changed: %s\n", Joystick_Name(d));

            char line[48];
            snprintf(line, sizeof(line), "joystick=%s\r\n", Joystick_Name(d));
            ble_uart_send_text(line);

            uint8_t payload[1] = { (uint8_t)d };
            int n = build_packet(MSG_ID_JOYSTICK, payload, 1, pkt, (int)sizeof(pkt));
            if (n > 0 && ble_client_ready()) {
                bool ok = ble_client_send(pkt, (uint16_t)n);
                if (ok) {
                    printf("TX -> STM: joystick=%s (%u bytes)\n", Joystick_Name(d), n);
                }
            }

            last = d;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    Joystick_Init();
    Joystick_StartTask();

    // Start BLE client (central) to connect to STM peripheral
    ble_client_init(STM_ADV_NAME, ble_ready_cb);

    xTaskCreate(sender_task, "sender", 4096, NULL, 5, NULL);

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
