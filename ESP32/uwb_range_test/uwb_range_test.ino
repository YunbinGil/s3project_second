// uwb_range_test.ino — UWB TWR 단독 디버그 스케치
//
// 역할: UART 프로토콜 없이 DWM1000 TWR 만 동작 확인
//       Serial 에서 "range" 입력 → motor.ino(차량)와 TWR → 거리 cm 출력
//
// ── 배선 ─────────────────────────────────────────────────────
//   DWM1000  SCK=18  MISO=19  MOSI=23  SS=5  RST=22
//   (UART 배선 불필요 — Serial(UART0) USB 시리얼만 사용)
//
// ── Arduino IDE 보드 설정 ────────────────────────────────────
//   도구 → 보드 → ESP32 Dev Module
//   시리얼 모니터 → 115200 baud, 줄 끝: Newline (LF)
//
// ── 사용법 ───────────────────────────────────────────────────
//   range  → TWR 1회 실행 → 거리 출력
//   info   → DWM1000 초기화 정보 출력

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgConstants.hpp>
#include <SPI.h>

// ── 핀 설정 (CLAUDE.md §4.2, §8) ────────────────────────────
#define UWB_PIN_SCK   18
#define UWB_PIN_MISO  19
#define UWB_PIN_MOSI  23
#define UWB_PIN_SS    5
#define UWB_PIN_RST   22

#define NODE_ID        2    // 이 노드의 Device Address
#define UWB_FRAME_LEN  16
#define RANGE_TIMEOUT_MS 300

// ── UWB 프레임 타입 (uart_comm.h 와 동일) ────────────────────
#define UWB_POLL         0
#define UWB_POLL_ACK     1
#define UWB_RANGE        2
#define UWB_RANGE_REPORT 3
#define UWB_RANGE_FAILED 255

// ── DWM1000 설정 (CLAUDE.md §8) ──────────────────────────────
device_configuration_t UWB_CONFIG = {
    false, true, true, true, false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_6800KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_128,
    PreambleCode::CODE_3
};

// ── 상태 머신 ────────────────────────────────────────────────
enum class RangeState : uint8_t {
    IDLE,
    WAIT_POLL_ACK,
    WAIT_RANGE_REPORT
};

static RangeState rangeState       = RangeState::IDLE;
static byte       uwbFrame[UWB_FRAME_LEN];
static uint64_t   timePollSent        = 0;
static uint64_t   timePollAckReceived = 0;
static uint32_t   deadlineMs          = 0;
static bool       debugMode           = false;  // 'debug' 명령으로 토글
static uint32_t   rxFrameCount        = 0;      // 수신된 전체 프레임 수

// ── POLL 전송 ────────────────────────────────────────────────
static void start_ranging() {
    if (rangeState != RangeState::IDLE) {
        Serial.println("[UWB] 이미 레인징 진행 중");
        return;
    }
    memset(uwbFrame, 0, sizeof(uwbFrame));
    uwbFrame[0] = UWB_POLL;
    DW1000Ng::forceTRxOff();                          // RX 모드 해제 후 TX 전환
    DW1000Ng::setTransmitData(uwbFrame, UWB_FRAME_LEN);
    DW1000Ng::startTransmit();
    rangeState = RangeState::WAIT_POLL_ACK;
    deadlineMs = millis() + RANGE_TIMEOUT_MS;
    if (debugMode) Serial.printf("[DBG] startTransmit(POLL) at %lu ms\n", millis());
    Serial.println("[UWB] POLL 전송 → POLL_ACK 대기...");
}

// ── RANGE 프레임 전송 (3개 타임스탬프 내포) ──────────────────
static void send_range() {
    memset(uwbFrame, 0, sizeof(uwbFrame));
    uwbFrame[0] = UWB_RANGE;

    // [fix] forceTRxOff 먼저: IDLE 확정 후 getSystemTimestamp() 호출
    // → DX_TIME 설정 전 chip 상태를 확실히 IDLE로 만들어 HPDWARN 방지
    DW1000Ng::forceTRxOff();

    // 지연 송신 시각 계산 (현재 + 5ms — SPI 오버헤드 고려 마진 확보)
    byte futureTimeBytes[LENGTH_TIMESTAMP];
    uint64_t timeRangeSent = DW1000Ng::getSystemTimestamp();
    timeRangeSent += DW1000NgTime::microsecondsToUWBTime(5000);
    DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
    DW1000Ng::setDelayedTRX(futureTimeBytes);
    timeRangeSent += DW1000Ng::getTxAntennaDelay();

    // 3개 타임스탬프 직렬화
    DW1000NgUtils::writeValueToBytes(uwbFrame + 1,  timePollSent,        LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(uwbFrame + 6,  timePollAckReceived, LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(uwbFrame + 11, timeRangeSent,       LENGTH_TIMESTAMP);

    DW1000Ng::setTransmitData(uwbFrame, UWB_FRAME_LEN);
    DW1000Ng::startTransmit(TransmitMode::DELAYED);
}

// ── 실패 처리 ────────────────────────────────────────────────
static void fail_ranging(const char *reason) {
    Serial.printf("[UWB] 측정 실패: %s  (총 수신 프레임: %lu)\n", reason, rxFrameCount);
    rangeState = RangeState::IDLE;
    DW1000Ng::forceTRxOff();
    DW1000Ng::startReceive();
    if (debugMode) Serial.printf("[DBG] startReceive() after fail at %lu ms\n", millis());
}

// ── UWB 폴링 FSM (loop() 에서 매 틱 호출) ───────────────────
// CLAUDE.md §8: isReceiveDone() 폴링, delay() 금지
static void process_uwb() {
    // 송신 완료 → 수신 모드 전환
    if (rangeState != RangeState::IDLE && DW1000Ng::isTransmitDone()) {
        DW1000Ng::clearTransmitStatus();
        DW1000Ng::startReceive();
        if (debugMode) Serial.printf("[DBG] TX done → startReceive() at %lu ms\n", millis());
    }

    // 타임아웃 체크
    if (rangeState != RangeState::IDLE &&
        (int32_t)(millis() - deadlineMs) > 0) {
        fail_ranging("timeout (차량 응답 없음)");
        return;
    }

    if (!DW1000Ng::isReceiveDone()) return;

    // 수신 데이터 검증
    const int dataLen = DW1000Ng::getReceivedDataLength();
    if (dataLen != UWB_FRAME_LEN) {
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive();
        if (rangeState != RangeState::IDLE) {
            fail_ranging("invalid frame len");
        }
        return;
    }

    byte frame[UWB_FRAME_LEN];
    DW1000Ng::getReceivedData(frame, UWB_FRAME_LEN);
    const byte msgId = frame[0];
    rxFrameCount++;

    if (debugMode) {
        Serial.printf("[DBG] RX frame #%lu  msgId=0x%02X  state=%d  ",
                      rxFrameCount, msgId, (int)rangeState);
        for (int i = 0; i < UWB_FRAME_LEN; i++) Serial.printf("%02X ", frame[i]);
        Serial.println();
    }

    // IDLE 중 예상치 못한 수신 → 무시하되 항상 출력
    if (rangeState == RangeState::IDLE) {
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive();
        Serial.printf("[UWB] IDLE 수신 #%lu  msgId=0x%02X ", rxFrameCount, msgId);
        for (int i = 0; i < UWB_FRAME_LEN; i++) Serial.printf("%02X ", frame[i]);
        Serial.println();
        return;
    }

    // ── WAIT_POLL_ACK ─────────────────────────────────────
    if (rangeState == RangeState::WAIT_POLL_ACK) {
        if (msgId != UWB_POLL_ACK) {
            DW1000Ng::clearReceiveStatus();
            DW1000Ng::startReceive();
            fail_ranging("expected POLL_ACK");
            return;
        }
        // clearReceiveStatus() 전에 타임스탬프 취득
        timePollSent        = DW1000Ng::getTransmitTimestamp();
        timePollAckReceived = DW1000Ng::getReceiveTimestamp();
        rangeState          = RangeState::WAIT_RANGE_REPORT;
        deadlineMs          = millis() + RANGE_TIMEOUT_MS;
        DW1000Ng::clearReceiveStatus();
        send_range();
        Serial.printf("[UWB] POLL_ACK 수신 → RANGE 전송  (rxCount=%lu)\n", rxFrameCount);
        return;
    }

    // ── WAIT_RANGE_REPORT ─────────────────────────────────
    if (rangeState == RangeState::WAIT_RANGE_REPORT) {
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive();

        if (msgId == UWB_RANGE_FAILED) {
            fail_ranging("차량(motor) 측 계산 실패");
            return;
        }
        if (msgId != UWB_RANGE_REPORT) {
            fail_ranging("expected RANGE_REPORT");
            return;
        }

        // motor.ino: frame[1..4] = float(distance * DISTANCE_OF_RADIO_INV)
        float rawDistance = 0.0f;
        memcpy(&rawDistance, frame + 1, sizeof(float));
        const float distanceMeters = rawDistance / DISTANCE_OF_RADIO_INV;
        const uint8_t carId        = frame[5];

        Serial.printf("[RANGE] Car%u : %.1f cm  (%.3f m)\n",
                      carId,
                      distanceMeters * 100.0f,
                      distanceMeters);
        rangeState = RangeState::IDLE;
    }
}

// ── setup ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("========================================");
    Serial.printf( "  UWB Range Test — Node%d\n", NODE_ID);
    Serial.println("  'range' 입력 → motor.ino 와 TWR");
    Serial.println("  'debug' 입력 → 상세 로그 토글");
    Serial.println("  'stat'  입력 → 수신 통계");
    Serial.println("  'info'  입력 → DWM1000 정보 출력");
    Serial.println("========================================");

    SPI.begin(UWB_PIN_SCK, UWB_PIN_MISO, UWB_PIN_MOSI, UWB_PIN_SS);
    DW1000Ng::initializeNoInterrupt(UWB_PIN_SS, UWB_PIN_RST);
    DW1000Ng::applyConfiguration(UWB_CONFIG);
    DW1000Ng::setNetworkId(10);
    DW1000Ng::setDeviceAddress((uint16_t)NODE_ID);
    DW1000Ng::setAntennaDelay(16436);
    DW1000Ng::startReceive();

    char buf[128];
    DW1000Ng::getPrintableDeviceIdentifier(buf);
    Serial.printf("  DW1000 ID: %s\n", buf);
    Serial.println("  DWM1000 초기화 완료 — 준비됨");
    Serial.println("========================================");
}

// ── loop ──────────────────────────────────────────────────────
void loop() {
    // Serial 명령 처리
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toLowerCase();

        if (cmd == "range") {
            start_ranging();
        } else if (cmd == "debug") {
            debugMode = !debugMode;
            Serial.printf("[INFO] debug 모드 %s\n", debugMode ? "ON" : "OFF");
        } else if (cmd == "stat") {
            Serial.printf("[STAT] 총 수신 프레임: %lu  현재 상태: %d  debugMode: %s\n",
                          rxFrameCount, (int)rangeState, debugMode ? "ON" : "OFF");
        } else if (cmd == "info") {
            char buf[128];
            DW1000Ng::getPrintableDeviceIdentifier(buf);
            Serial.printf("[INFO] Device ID: %s\n", buf);
            DW1000Ng::getPrintableNetworkIdAndShortAddress(buf);
            Serial.printf("[INFO] Network/Addr: %s\n", buf);
            DW1000Ng::getPrintableDeviceMode(buf);
            Serial.printf("[INFO] Mode: %s\n", buf);
        } else if (cmd.length() > 0) {
            Serial.println("[INFO] 명령: range | debug | stat | info");
        }
    }

    // UWB FSM 폴링
    process_uwb();
}
