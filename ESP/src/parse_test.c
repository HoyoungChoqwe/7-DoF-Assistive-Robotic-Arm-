// #include <stdio.h>
// #include <stdint.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "packet_parser.h"

// static int g_ok_count = 0;

// static void on_packet(const Packet *pkt, void *user)
// {
//     (void)user;
//     g_ok_count++;

//     printf("RX OK #%d: ID=0x%02X LEN=%u PAYLOAD=", g_ok_count, pkt->id, pkt->len);
//     for (uint8_t i = 0; i < pkt->len; i++) printf("%02X ", pkt->payload[i]);
//     printf("CHK=0x%02X\n", pkt->checksum);
// }

// static uint8_t chk_calc(uint8_t len, uint8_t id, const uint8_t *payload)
// {
//     uint16_t sum = (uint16_t)len + (uint16_t)id;
//     for (uint8_t i = 0; i < len; i++) sum += payload[i];
//     return (uint8_t)(sum & 0xFF);
// }

// static void feed(PacketParser *pp, const uint8_t *data, int n)
// {
//     for (int i = 0; i < n; i++) {
//         ParserResult r = PacketParser_PushByte(pp, data[i]);
//         if (r == PARSER_ERR_LEN) printf("PARSER_ERR_LEN\n");
//         if (r == PARSER_ERR_CHK) printf("PARSER_ERR_CHK\n");
//     }
// }

// static void run_parser_tests(void)
// {
//     PacketParser pp;
//     PacketParser_Init(&pp, on_packet, NULL);

//     // --- Test 1: valid packet ---
//     uint8_t p1_payload[] = {0x10, 0x20, 0x30};
//     uint8_t pkt1[] = {
//         PKT_SOF1, PKT_SOF2,
//         3, 0x01,
//         0x10, 0x20, 0x30,
//         0x00
//     };
//     pkt1[sizeof(pkt1) - 1] = chk_calc(3, 0x01, p1_payload);

//     printf("\n[Test 1] Valid packet: expect 1 RX OK\n");
//     int ok_before = g_ok_count;
//     feed(&pp, pkt1, (int)sizeof(pkt1));
//     printf("OK delta = %d\n", g_ok_count - ok_before);

//     // --- Test 2: bad checksum ---
//     uint8_t pkt2[sizeof(pkt1)];
//     for (size_t i = 0; i < sizeof(pkt1); i++) pkt2[i] = pkt1[i];
//     pkt2[sizeof(pkt2) - 1] ^= 0xFF;

//     printf("\n[Test 2] Bad checksum: expect PARSER_ERR_CHK and 0 RX OK\n");
//     ok_before = g_ok_count;
//     feed(&pp, pkt2, (int)sizeof(pkt2));
//     printf("OK delta = %d\n", g_ok_count - ok_before);

//     // --- Test 3: noise then valid (resync) ---
//     uint8_t noise[] = {0x00, 0xFF, 0x12, 0xAA, 0x00, 0x99, 0x55};
//     printf("\n[Test 3] Noise then valid: expect 1 RX OK\n");
//     ok_before = g_ok_count;
//     feed(&pp, noise, (int)sizeof(noise));
//     feed(&pp, pkt1, (int)sizeof(pkt1));
//     printf("OK delta = %d\n", g_ok_count - ok_before);

//     // --- Test 4: too-long length ---
//     printf("\n[Test 4] LEN too big: expect PARSER_ERR_LEN\n");
//     uint8_t pkt4[] = { PKT_SOF1, PKT_SOF2, (uint8_t)(PKT_MAX_PAYLOAD + 1), 0x02 };
//     feed(&pp, pkt4, (int)sizeof(pkt4));

//     printf("\nDone. Total OK packets = %d\n\n", g_ok_count);
// }

// void app_main(void)
// {
//     run_parser_tests();
//     while (1) vTaskDelay(pdMS_TO_TICKS(1000));
// }