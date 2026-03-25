#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

namespace proto {

// ── 프레임 구분자 ──────────────────────────────────────────────
static constexpr uint8_t STX     = 0x02;   // Start of frame
static constexpr uint8_t ETX     = 0x03;   // End of frame
static constexpr uint8_t DLE     = 0x10;   // Data Link Escape (byte stuffing)
static constexpr uint8_t DLE_XOR = 0x20;   // DLE stuffing XOR mask

// ── 커맨드 ────────────────────────────────────────────────────
static constexpr uint8_t CMD_PING = 0x50;  // RPi → ESP32
static constexpr uint8_t CMD_PONG = 0x51;  // ESP32 → RPi
static constexpr uint8_t CMD_RANGE_REPORT = 0x52;   // ESP32 → RPi
static constexpr uint8_t CMD_CTRL_FWD = 0x54;       // RPi → ESP32
static constexpr uint8_t CMD_MODE_NOTIFY = 0x55;    // ESP32 → RPi
static constexpr uint8_t CMD_RANGE_REQUEST = 0x56;  // RPi → ESP32

// ── 버퍼 제한 ─────────────────────────────────────────────────
static constexpr size_t MAX_PAYLOAD = 64;
static constexpr size_t MAX_FRAME   = (MAX_PAYLOAD + 3) * 2 + 2; // 최대 stuffed 크기

// ── CRC-8 (CCITT, poly=0x07, 256-byte 룩업 테이블) ────────────
uint8_t crc8(const uint8_t *data, size_t len);

// ── 프레임 빌더 ───────────────────────────────────────────────
// payload: 전송할 원시 데이터 포인터, len: 바이트 수
// 반환: [STX][DLE-stuffed([LEN][PAYLOAD...][CRC8])][ETX]
std::vector<uint8_t> build_frame(const uint8_t *payload, uint8_t len);

// ── FSM 파서 ──────────────────────────────────────────────────
enum class ParseState : uint8_t {
    WAIT_STX,   // STX 대기
    RAW_RECV,   // unstuffed 바이트 수집 중
    DLE_RECV,   // DLE 다음 바이트 대기
};

struct Parser {
    ParseState state   = ParseState::WAIT_STX;
    uint8_t    raw[MAX_PAYLOAD + 3]; // [LEN][PAYLOAD...][CRC8] unstuffed 버퍼
    size_t     raw_pos = 0;

    // 바이트 1개씩 투입. 완전한 유효 패킷 수신 시 true 반환.
    // true 반환 시 payload_out[0..payload_len-1] 에 페이로드 기록됨.
    bool feed(uint8_t byte, uint8_t *payload_out, uint8_t *payload_len_out);

    void reset();
};

} // namespace proto
