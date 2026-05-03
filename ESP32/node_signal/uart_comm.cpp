#include "uart_comm.h"
#include <string.h>

void UartComm::begin() {
    proto_parser_reset(&_parser);
    _serial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.printf("[UART] Node%d TX=GPIO%d RX=GPIO%d %d baud\n",
                  NODE_ID, UART_TX_PIN, UART_RX_PIN, UART_BAUD);
}

void UartComm::process() {
    while (_serial.available()) {
        uint8_t b = (uint8_t)_serial.read();
        uint8_t payload[PROTO_MAX_PAYLOAD];
        uint8_t plen = 0;
        if (proto_parser_feed(&_parser, b, payload, &plen)) {
            handle_packet(payload, plen);
        }
    }
}

void UartComm::handle_packet(const uint8_t* payload, uint8_t len) {
    if (len < 1) return;

    // ── PING → PONG ──────────────────────────
    if (payload[0] == CMD_PING && len >= 3) {
        uint16_t seq = ((uint16_t)payload[1] << 8) | payload[2];
        Serial.printf("[UART] PING recv SEQ=%u\n", seq);

        uint8_t pong[4];
        pong[0] = CMD_PONG;
        pong[1] = (seq >> 8) & 0xFF;
        pong[2] =  seq       & 0xFF;
        pong[3] = (uint8_t)NODE_ID;
        send(pong, sizeof(pong));
        Serial.printf("[UART] PONG sent SEQ=%u\n", seq);
    }

    // ── CMD_CTRL_FWD → 콜백으로 전달 ─────────
    else if (payload[0] == CMD_CTRL_FWD && len >= 4) {
        uint8_t topic_len = payload[1];
        if (len < 2u + topic_len + 2u) return;

        char topic[32] = {0};
        uint8_t copy = (topic_len < 31) ? topic_len : 31;
        memcpy(topic, payload + 2, copy);

        uint16_t plen = ((uint16_t)payload[2 + topic_len] << 8)
                      |  (uint16_t)payload[3 + topic_len];
        if (len < 2u + topic_len + 2u + plen) return;

        const uint8_t* pdata = payload + 2 + topic_len + 2;
        Serial.printf("[UART] CTRL_FWD topic=%s len=%u\n", topic, plen);

        if (_ctrlFwdCb) _ctrlFwdCb(topic, pdata, (uint8_t)plen);
    }

    else {
        Serial.printf("[UART] unknown CMD=0x%02X\n", payload[0]);
    }
}

void UartComm::send(const uint8_t* payload, uint8_t len) {
    uint8_t frame[PROTO_MAX_FRAME];
    size_t flen = proto_build_frame(payload, len, frame, sizeof(frame));
    if (flen == 0) { Serial.println("[UART] frame build failed"); return; }
    _serial.write(frame, flen);
}
