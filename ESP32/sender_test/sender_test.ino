// sender_test.ino — node_template 동작 검증용 테스트 스케치
//
// 역할: RPi를 대신해 UART로 CMD_RANGE_REQUEST 를 전송하고,
//       node_template 이 UWB TWR 후 돌려주는 CMD_RANGE_REPORT 를
//       시리얼 모니터에 출력한다.
//
// ── 배선 (크로스 연결 필수) ──────────────────────────────────
//
//   sender_test ESP32                node_template ESP32
//   ─────────────────────            ─────────────────────────
//   GPIO17 (TXD2, UART2 TX) ──────→  GPIO16 (RXD2, UART_RX_PIN)
//   GPIO16 (RXD2, UART2 RX) ←──────  GPIO17 (TXD2, UART_TX_PIN)
//   GND                ─────────── GND
//
//   양쪽 모두 GPIO16=RX, GPIO17=TX (UART2 RXD2/TXD2) 로 통일
//   DWM1000(SS=5, RST=22)는 sender_test에 없으므로 GPIO16/17 충돌 없음
//
// ── Arduino IDE 보드 설정 ────────────────────────────────────
//   도구 → 보드 → esp32 by Espressif → ESP32 Dev Module
//   도구 → 포트 → sender_test 가 연결된 COM 포트
//   시리얼 모니터 → 115200 baud, 줄 끝: Newline (LF)
//
// ── 사용법 ───────────────────────────────────────────────────
//   range   → CMD_RANGE_REQUEST 1회 전송 → 거리(cm) 출력
//   ping    → CMD_PING 전송 → PONG 응답 확인
//   autoN   → N초 간격 자동 range 요청 (예: auto3 → 3초 간격)
//   stop    → 자동 요청 중지

#include "protocol.h"

// ── 핀 설정 ──────────────────────────────────────────────────
// node_template 과 동일하게 GPIO17=TXD2, GPIO16=RXD2 (UART2) 사용
// sender GPIO17 (TX) → node_template GPIO16 (RX)
// sender GPIO16 (RX) ← node_template GPIO17 (TX)
#define SENDER_TX_PIN  17    // TXD2
#define SENDER_RX_PIN  16    // RXD2
#define SENDER_BAUD    115200
#define TARGET_NODE_ID 2      // 연결 대상 node_template 의 NODE_ID (uart_comm.h 와 일치시킬 것)

// ── 내부 상태 ─────────────────────────────────────────────────
static HardwareSerial NodeSerial(2);   // UART2 — GPIO16(RXD2)/GPIO17(TXD2) 기본 핀
static ProtoParser    parser;
static uint16_t       seqCounter = 0;

static uint32_t autoIntervalMs = 0;    // 0 = 자동 비활성
static uint32_t lastAutoMs     = 0;

// ── 패킷 송신 헬퍼 ────────────────────────────────────────────
static void send_packet(const uint8_t *payload, uint8_t len) {
    uint8_t frame[PROTO_MAX_FRAME];
    size_t flen = proto_build_frame(payload, len, frame, sizeof(frame));
    if (flen > 0) {
        NodeSerial.write(frame, flen);
    }
}

static void send_range_request() {
    uint16_t seq = ++seqCounter;
    uint8_t payload[3];
    payload[0] = CMD_RANGE_REQUEST;
    payload[1] = (seq >> 8) & 0xFF;   // 빅 엔디안 직렬화 (CLAUDE.md §5)
    payload[2] =  seq       & 0xFF;
    send_packet(payload, sizeof(payload));
    Serial.printf("[TX] CMD_RANGE_REQUEST → Node%d  SEQ=%u\n", TARGET_NODE_ID, seq);
}

static void send_ping() {
    uint16_t seq = ++seqCounter;
    uint8_t payload[3];
    payload[0] = CMD_PING;
    payload[1] = (seq >> 8) & 0xFF;
    payload[2] =  seq       & 0xFF;
    send_packet(payload, sizeof(payload));
    Serial.printf("[TX] CMD_PING → Node%d  SEQ=%u\n", TARGET_NODE_ID, seq);
}

// ── 수신 패킷 처리 ────────────────────────────────────────────
static void handle_packet(const uint8_t *payload, uint8_t len) {
    if (len < 1) return;

    switch (payload[0]) {

    // ── CMD_RANGE_REPORT (0x52) ─────────────────────────────
    // 페이로드: [NODE_ID][car_id][DIST_HI][DIST_LO][RSSI_HI][RSSI_LO]
    case CMD_RANGE_REPORT:
        if (len >= 7) {
            const uint8_t  nodeId    = payload[1];
            const uint8_t  carId     = payload[2];
            const uint16_t distCm    = ((uint16_t)payload[3] << 8) | payload[4];
            const int16_t  rssiCenti = ((int16_t) payload[5] << 8) | payload[6];

            if (distCm == 0xFFFF) {
                Serial.printf("[RANGE] Node%u → 측정 실패 (TWR timeout / 범위 초과)\n",
                              nodeId);
            } else {
                Serial.printf("[RANGE] Node%u → Car%u : %4u cm  (%.2f m)  "
                              "RSSI = %.2f dBm\n",
                              nodeId, carId,
                              distCm,
                              distCm / 100.0f,
                              rssiCenti / 100.0f);
            }
        } else {
            Serial.printf("[RANGE] 페이로드 길이 오류 (len=%u, 최소 7 필요)\n", len);
        }
        break;

    // ── CMD_PONG (0x51) ─────────────────────────────────────
    case CMD_PONG:
        if (len >= 4) {
            const uint16_t seq    = ((uint16_t)payload[1] << 8) | payload[2];
            const uint8_t  nodeId = payload[3];
            Serial.printf("[PONG]  Node%u 응답  SEQ=%u  → 연결 정상\n", nodeId, seq);
        }
        break;

    // ── CMD_MODE_NOTIFY (0x55) — 차량 FSM 모드 전환 알림 ────
    case CMD_MODE_NOTIFY:
        if (len >= 3) {
            const uint8_t nodeId = payload[1];
            const uint8_t mode   = payload[2];
            Serial.printf("[MODE]  Node%u 모드전환: %s\n",
                          nodeId,
                          mode == 1 ? "MQTT_CONTROL" : "HTTP_CONTROL");
        }
        break;

    default:
        Serial.printf("[RX] 알 수 없는 CMD=0x%02X  len=%u\n", payload[0], len);
        break;
    }
}

// ── 명령어 파싱 ───────────────────────────────────────────────
static void process_serial_command(const String &cmd) {
    if (cmd == "range") {
        send_range_request();

    } else if (cmd == "ping") {
        send_ping();

    } else if (cmd.startsWith("auto")) {
        // "auto3" → 3초 간격, "auto0" 또는 "auto" → 중지
        String numStr = cmd.substring(4);
        uint32_t sec = numStr.toInt();
        if (sec == 0) {
            autoIntervalMs = 0;
            Serial.println("[AUTO] 자동 측정 중지");
        } else {
            autoIntervalMs = sec * 1000UL;
            lastAutoMs     = millis() - autoIntervalMs; // 즉시 1회 실행
            Serial.printf("[AUTO] %u 초 간격 자동 측정 시작\n", sec);
        }

    } else if (cmd == "stop") {
        autoIntervalMs = 0;
        Serial.println("[AUTO] 자동 측정 중지");

    } else if (cmd.length() > 0) {
        Serial.println("[INFO] 명령 목록: range | ping | autoN (예: auto3) | stop");
    }
}

// ── setup ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("========================================");
    Serial.println("  sender_test — node_template 테스트");
    Serial.printf( "  대상 노드: Node%d\n", TARGET_NODE_ID);
    Serial.printf( "  UART2  TXD2=GPIO%d  RXD2=GPIO%d  %d baud\n",
                   SENDER_TX_PIN, SENDER_RX_PIN, SENDER_BAUD);
    Serial.println("========================================");
    Serial.println("명령어:");
    Serial.println("  range       → 거리 1회 측정");
    Serial.println("  ping        → 연결 확인");
    Serial.println("  auto3       → 3초 간격 자동 측정 (숫자 변경 가능)");
    Serial.println("  stop        → 자동 측정 중지");
    Serial.println("========================================");

    proto_parser_reset(&parser);
    NodeSerial.begin(SENDER_BAUD, SERIAL_8N1, SENDER_RX_PIN, SENDER_TX_PIN);
}

// ── loop ──────────────────────────────────────────────────────
void loop() {
    // ── 시리얼 모니터 입력 처리 ──────────────────────────────
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toLowerCase();
        process_serial_command(cmd);
    }

    // ── 자동 측정 타이머 ─────────────────────────────────────
    if (autoIntervalMs > 0 &&
        (millis() - lastAutoMs) >= autoIntervalMs) {
        lastAutoMs = millis();
        send_range_request();
    }

    // ── node_template 에서 수신한 데이터 처리 ────────────────
    // CLAUDE.md §4: available() → read() → FSM 투입, delay() 없음
    while (NodeSerial.available()) {
        uint8_t b = (uint8_t)NodeSerial.read();
        uint8_t payload[PROTO_MAX_PAYLOAD];
        uint8_t plen = 0;
        if (proto_parser_feed(&parser, b, payload, &plen)) {
            handle_packet(payload, plen);
        }
    }
}
