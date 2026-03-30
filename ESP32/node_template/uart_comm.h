#pragma once

#include <HardwareSerial.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include "protocol.h"

// ── 노드별 설정 — 업로드 전에 반드시 노드에 맞게 수정 ──────────
// Node1: NODE_ID=1 / Node2: NODE_ID=2 / Node3: NODE_ID=3
#define NODE_ID      2
#define UART_TX_PIN  17     // TXD2 (GPIO17) — RPi RX에 연결
#define UART_RX_PIN  16     // RXD2 (GPIO16) — RPi TX에 연결
#define UART_BAUD    115200

#define UWB_PIN_SCK   18
#define UWB_PIN_MISO  19
#define UWB_PIN_MOSI  23
#define UWB_PIN_SS    5
#define UWB_PIN_RST   22
#define UWB_PIN_IRQ   4

#define UWB_FRAME_LEN 16
#define UWB_POLL 0
#define UWB_POLL_ACK 1
#define UWB_RANGE 2
#define UWB_RANGE_REPORT 3
#define UWB_RANGE_FAILED 255
#define UWB_MODE_NOTIFY  0xF1   // motor → node: FSM 모드 전환 알림 (16바이트 프레임)
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
    // UART0(Serial)는 USB 디버그 전용 — GPIO16(RXD2)/GPIO17(TXD2)는 UART2 기본 핀
    HardwareSerial _serial{2};
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
    void report_range(uint8_t car_id, uint16_t distance_cm, int16_t rssi_centi_dbm);
    void fail_ranging(const char *reason);
    static uint16_t meters_to_cm(float meters);

    // CMD_CTRL_FWD: Proto-B UWB 패킷 빌드 후 motor 로 전송
    void send_ctrl_fwd(const uint8_t* topic, uint8_t topic_len,
                       const uint8_t* payload, uint8_t payload_len);
    // CRC-16 CCITT (poly=0x1021, init=0xFFFF) — motor.ino / sender.ino 와 동일
    static uint16_t calc_crc16(const uint8_t* data, uint16_t len);

    uint16_t _ctrlMsgId = 1;  // Proto-B 메시지 ID 카운터

};
