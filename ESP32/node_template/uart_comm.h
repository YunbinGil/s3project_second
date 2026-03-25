#pragma once

#include <HardwareSerial.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include "protocol.h"

// ── 노드별 설정 — 업로드 전에 반드시 노드에 맞게 수정 ──────────
// Node1: NODE_ID=1 / Node2: NODE_ID=2 / Node3: NODE_ID=3
#define NODE_ID      1
#define UART_TX_PIN  25     // 라즈베리파이 RX 에 연결된 GPIO
#define UART_RX_PIN  26     // 라즈베리파이 TX 에 연결된 GPIO
#define UART_BAUD    115200

#define UWB_PIN_SCK   18
#define UWB_PIN_MISO  19
#define UWB_PIN_MOSI  23
#define UWB_PIN_SS    17
#define UWB_PIN_RST   16
#define UWB_PIN_IRQ   4

#define UWB_FRAME_LEN 16
#define UWB_POLL 0
#define UWB_POLL_ACK 1
#define UWB_RANGE 2
#define UWB_RANGE_REPORT 3
#define UWB_RANGE_FAILED 255
// ──────────────────────────────────────────────────────────────

// ── UartComm 클래스 ───────────────────────────────────────────
// HardwareSerial(1) → UART1 (NOT Serial/UART0, CLAUDE.md §4)
// loop() 에서 process() 를 매 틱마다 호출
class UartComm {
public:
    // setup() 에서 1회 호출
    void begin();

    // loop() 에서 매 틱마다 호출 — 수신 바이트를 FSM 파서에 투입
    // delay() 없이 즉시 반환 (CLAUDE.md §4)
    void process();

    // 페이로드를 프레임으로 감싸 UART1 로 송신
    void send(const uint8_t *payload, uint8_t len);

private:
    // UART0(Serial)는 USB 디버그 전용 — 반드시 UART1 사용
    HardwareSerial _serial{1};
    ProtoParser    _parser;

    enum class RangeState : uint8_t {
        IDLE,
        WAIT_POLL_ACK,
        WAIT_RANGE_REPORT,
    };

    byte _uwbData[UWB_FRAME_LEN] = {0};
    uint64_t _timePollSent = 0;
    uint64_t _timePollAckReceived = 0;
    RangeState _rangeState = RangeState::IDLE;
    uint16_t _activeSeq = 0;
    uint32_t _rangeDeadlineMs = 0;

    void handle_packet(const uint8_t *payload, uint8_t len);
    void process_uwb();
    void start_ranging(uint16_t seq);
    void send_poll();
    void send_range();
    void report_range(uint16_t distance_cm, int16_t rssi_centi_dbm);
    void fail_ranging(const char *reason);
    static uint16_t meters_to_cm(float meters);

};
