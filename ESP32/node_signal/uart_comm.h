#pragma once
#include <HardwareSerial.h>
#include "protocol.h"

// ── 노드별 설정 — 업로드 전 NODE_ID 수정 ────
#define NODE_ID      1       // 신호등 1: 1 / 2: 2 / 3: 3
#define UART_TX_PIN  25      // RPi RX에 연결
#define UART_RX_PIN  26      // RPi TX에 연결
#define UART_BAUD    115200

// ── 콜백 타입 ────────────────────────────────
// RPi에서 CMD_CTRL_FWD 수신 시 호출
// topic: "command/led" 등, payload: 데이터, len: 길이
typedef void (*CtrlFwdCallback)(const char* topic, const uint8_t* payload, uint8_t len);

class UartComm {
public:
    void begin();
    void process();
    void send(const uint8_t* payload, uint8_t len);
    void setCtrlFwdCallback(CtrlFwdCallback cb) { _ctrlFwdCb = cb; }

private:
    HardwareSerial _serial{1};   // UART1
    ProtoParser    _parser;
    CtrlFwdCallback _ctrlFwdCb = nullptr;

    void handle_packet(const uint8_t* payload, uint8_t len);
};
