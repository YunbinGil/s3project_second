#pragma once

#include <Arduino.h>

// ── 프레임 구분자 ──────────────────────────────────────────────
#define PROTO_STX      (0x02u)   // Start of frame
#define PROTO_ETX      (0x03u)   // End of frame
#define PROTO_DLE      (0x10u)   // Data Link Escape (byte stuffing)
#define PROTO_DLE_XOR  (0x20u)   // DLE stuffing XOR mask

// ── 커맨드 ────────────────────────────────────────────────────
#define CMD_PING  (0x50u)   // RPi → ESP32
#define CMD_PONG  (0x51u)   // ESP32 → RPi
#define CMD_RANGE_REPORT  (0x52u)   // ESP32 → RPi
#define CMD_CTRL_FWD      (0x54u)   // RPi → ESP32
#define CMD_MODE_NOTIFY   (0x55u)   // ESP32 → RPi
#define CMD_RANGE_REQUEST (0x56u)   // RPi → ESP32

// ── 버퍼 제한 ─────────────────────────────────────────────────
#define PROTO_MAX_PAYLOAD  64u
#define PROTO_MAX_FRAME    ((PROTO_MAX_PAYLOAD + 3u) * 2u + 2u)

// ── CRC-8 (CCITT, poly=0x07, 256-byte 룩업 테이블) ────────────
// RPi 측 C++ 코드와 동일한 테이블 — 이기종 간 CRC 호환 보장
uint8_t proto_crc8(const uint8_t *data, size_t len);

// ── 프레임 빌더 ───────────────────────────────────────────────
// payload: 전송 데이터, len: 바이트 수
// frame_out: 출력 버퍼 (PROTO_MAX_FRAME 이상 크기 필요)
// 반환값: 기록된 바이트 수 (0 = 오류)
size_t proto_build_frame(const uint8_t *payload, uint8_t len,
                         uint8_t *frame_out, size_t frame_buf_size);

// ── FSM 파서 ──────────────────────────────────────────────────
typedef enum {
    PARSE_WAIT_STX = 0,
    PARSE_RAW_RECV,
    PARSE_DLE_RECV,
} ProtoParseState;

typedef struct {
    ProtoParseState state;
    uint8_t raw[PROTO_MAX_PAYLOAD + 3];   // [LEN][PAYLOAD...][CRC8]
    size_t  raw_pos;
} ProtoParser;

void proto_parser_reset(ProtoParser *p);

// 바이트 하나 투입. 완전한 패킷 수신 시 true 반환.
// payload_out: 결과 페이로드 버퍼 (PROTO_MAX_PAYLOAD 이상 크기)
bool proto_parser_feed(ProtoParser *p, uint8_t byte,
                       uint8_t *payload_out, uint8_t *payload_len_out);
