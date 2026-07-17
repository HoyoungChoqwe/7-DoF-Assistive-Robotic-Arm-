#ifndef PACKET_PARSER_H
#define PACKET_PARSER_H

#include <stdint.h>
#include <stddef.h>

#ifndef PKT_SOF1
#define PKT_SOF1 0xAA
#endif

#ifndef PKT_SOF2
#define PKT_SOF2 0x55
#endif

#ifndef PKT_MAX_PAYLOAD
#define PKT_MAX_PAYLOAD 64
#endif

typedef struct {
    uint8_t len;                       // payload length
    uint8_t id;                        // message ID
    uint8_t payload[PKT_MAX_PAYLOAD];  // payload bytes
    uint8_t checksum;                  // received checksum byte
} Packet;

typedef enum {
    PARSER_OK = 0,
    PARSER_IN_PROGRESS,
    PARSER_PACKET_READY,
    PARSER_ERR_LEN,
    PARSER_ERR_CHK
} ParserResult;

typedef void (*packet_rx_cb_t)(const Packet *pkt, void *user);

typedef struct {
    // state machine
    enum { S_SOF1, S_SOF2, S_LEN, S_ID, S_PAYLOAD, S_CHK } state;

    Packet cur;
    uint8_t index;
    uint8_t chk;

    packet_rx_cb_t cb;
    void *user;
} PacketParser;

void PacketParser_Init(PacketParser *p, packet_rx_cb_t cb, void *user);

// Push one byte into the parser.
// Returns PARSER_PACKET_READY when a packet is completed (and callback has run).
ParserResult PacketParser_PushByte(PacketParser *p, uint8_t b);

// Optional: reset parser to initial state
void PacketParser_Reset(PacketParser *p);

#endif