#include "packet_parser.h"

// checksum over LEN, ID, PAYLOAD (8-bit additive)
static uint8_t pkt_checksum_calc(uint8_t len, uint8_t id, const uint8_t *payload)
{
    uint16_t sum = 0;
    sum += len;
    sum += id;
    for (uint8_t i = 0; i < len; i++) sum += payload[i];
    return (uint8_t)(sum & 0xFF);
}

void PacketParser_Reset(PacketParser *p)
{
    p->state = S_SOF1;
    p->index = 0;
    p->chk = 0;
}

void PacketParser_Init(PacketParser *p, packet_rx_cb_t cb, void *user)
{
    p->cb = cb;
    p->user = user;
    PacketParser_Reset(p);
}

ParserResult PacketParser_PushByte(PacketParser *p, uint8_t b)
{
    switch (p->state) {
    case S_SOF1:
        if (b == PKT_SOF1) p->state = S_SOF2;
        return PARSER_IN_PROGRESS;

    case S_SOF2:
        if (b == PKT_SOF2) {
            p->state = S_LEN;
        } else if (b == PKT_SOF1) {
            // stay in SOF2 hunt if repeated SOF1
            p->state = S_SOF2;
        } else {
            p->state = S_SOF1;
        }
        return PARSER_IN_PROGRESS;

    case S_LEN:
        p->cur.len = b;
        if (p->cur.len > PKT_MAX_PAYLOAD) {
            PacketParser_Reset(p);
            return PARSER_ERR_LEN;
        }
        p->state = S_ID;
        return PARSER_IN_PROGRESS;

    case S_ID:
        p->cur.id = b;
        p->index = 0;
        p->state = (p->cur.len == 0) ? S_CHK : S_PAYLOAD;
        return PARSER_IN_PROGRESS;

    case S_PAYLOAD:
        p->cur.payload[p->index++] = b;
        if (p->index >= p->cur.len) {
            p->state = S_CHK;
        }
        return PARSER_IN_PROGRESS;

    case S_CHK: {
        p->cur.checksum = b;
        uint8_t expect = pkt_checksum_calc(p->cur.len, p->cur.id, p->cur.payload);
        if (expect != p->cur.checksum) {
            PacketParser_Reset(p);
            return PARSER_ERR_CHK;
        }

        // valid packet
        if (p->cb) p->cb(&p->cur, p->user);
        PacketParser_Reset(p);
        return PARSER_PACKET_READY;
    }

    default:
        PacketParser_Reset(p);
        return PARSER_IN_PROGRESS;
    }
}