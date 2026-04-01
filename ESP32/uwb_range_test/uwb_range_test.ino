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
#define RANGE_TIMEOUT_MS    5000  // 디버그: 5초로 확대 (node_car 응답 대기)
#define RANGE_SEND_DELAY_MS  10    // node_car processMessages() 완료 대기

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
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

// ── 상태 머신 ────────────────────────────────────────────────
enum class RangeState : uint8_t {
    IDLE,
    WAIT_POLL_ACK,
    PENDING_RANGE_SEND,   // POLL_ACK 수신 후 node_car 준비 대기
    WAIT_RANGE_REPORT
};

static RangeState rangeState       = RangeState::IDLE;
static byte       uwbFrame[UWB_FRAME_LEN];
static uint64_t   timePollSent        = 0;
static uint64_t   timePollAckReceived = 0;
static uint32_t   deadlineMs          = 0;
static uint32_t   rangeSendAt         = 0;  // PENDING_RANGE_SEND: RANGE 전송 예약 시각
static bool       debugMode           = false;  // 'debug' 명령으로 토글
static uint32_t   rxFrameCount        = 0;      // 수신된 전체 프레임 수
static bool       pendingRetry        = false;  // 자동 재시도 예약
static uint32_t   retryAt             = 0;      // 재시도 시각
static int        retryCount          = 0;      // 현재 시도 횟수
const  int        MAX_RETRY           = 1;      // 최대 자동 재시도 횟수

// ── 이동 평균 필터 (창 크기 3) ────────────────────────────────
// 극단값 완화: UWB 인도어 멀티패스/노이즈로 인한 단발성 이상값 억제
#define RANGE_FILTER_SIZE  3
static float  rangeHistory[RANGE_FILTER_SIZE] = {-1.0f, -1.0f, -1.0f};
static int    rangeHistIdx = 0;

static float applyRangeFilter(float newSample) {
    rangeHistory[rangeHistIdx] = newSample;
    rangeHistIdx = (rangeHistIdx + 1) % RANGE_FILTER_SIZE;

    // 유효한(≥0) 샘플만 평균
    float sum = 0.0f;
    int   cnt = 0;
    for (int i = 0; i < RANGE_FILTER_SIZE; i++) {
        if (rangeHistory[i] >= 0.0f) { sum += rangeHistory[i]; cnt++; }
    }
    return (cnt > 0) ? (sum / cnt) : newSample;
}

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
}

// ── RANGE 프레임 전송 (3개 타임스탬프 내포) ──────────────────
// Delayed TX 방식:
//   forceTRxOff() 를 setDelayedTRX() 보다 먼저 호출해야 함 (이전 버그 수정)
//   1500µs 지연 = SPI 오버헤드(≈300µs) + 안전 마진
//   Delayed TX 는 정확한 timeRangeSent 를 미리 알 수 있어 계산 오차↓
//   (IMMEDIATE 방식은 실제 TX 시각과 수십~수백µs 오차 → 거리 ±수십m)
static void send_range() {
    DW1000Ng::forceTRxOff();  // ← 반드시 setDelayedTRX() 전에 호출

    byte futureTimeBytes[LENGTH_TIMESTAMP];
    uint64_t timeRangeSent = DW1000Ng::getSystemTimestamp();
    timeRangeSent += DW1000NgTime::microsecondsToUWBTime(3000);  // 3ms 후 발사 (SPI 오버헤드 고려)
    DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
    DW1000Ng::setDelayedTRX(futureTimeBytes);                    // ← forceTRxOff 이후
    timeRangeSent += DW1000Ng::getTxAntennaDelay();              // 정확한 TX 시각

    memset(uwbFrame, 0, sizeof(uwbFrame));
    uwbFrame[0] = UWB_RANGE;
    DW1000NgUtils::writeValueToBytes(uwbFrame + 1,  timePollSent,        LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(uwbFrame + 6,  timePollAckReceived, LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(uwbFrame + 11, timeRangeSent,       LENGTH_TIMESTAMP);

    DW1000Ng::setTransmitData(uwbFrame, UWB_FRAME_LEN);
    // Serial.println은 startTransmit 이후에 호출 — 전에 호출하면 수ms 지연으로
    // getSystemTimestamp()+1500µs 예약 시각이 경과 → HPDWARN → TX 취소
    DW1000Ng::startTransmit(TransmitMode::DELAYED);
    Serial.println("[UWB] RANGE startTransmit(DELAYED 3000µs) 완료");

    // 3ms 발사 대기 + 마진 50ms
    const uint32_t txDeadline = millis() + 100;
    while (!DW1000Ng::isTransmitDone()) {
        if ((int32_t)(millis() - txDeadline) > 0) {
            Serial.println("[UWB] RANGE TX 타임아웃 — HPDWARN 의심 (delay 부족, 3000µs 초과)");
            fail_ranging("RANGE TX 실패");
            return;
        }
    }
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::startReceive();
}

// ── 실패 처리 ────────────────────────────────────────────────
static void fail_ranging(const char *reason) {
    Serial.printf("[UWB] 측정 실패: %s  (총 수신 프레임: %lu)\n", reason, rxFrameCount);
    rangeState = RangeState::IDLE;
    DW1000Ng::forceTRxOff();
    DW1000Ng::startReceive();
    // 자동 재시도 예약 (500ms 후)
    if (retryCount < MAX_RETRY) {
        retryCount++;
        pendingRetry = true;
        retryAt = millis() + 500;
        Serial.printf("[UWB] 500ms 후 자동 재시도 %d/%d\n", retryCount, MAX_RETRY);
    } else {
        Serial.printf("[UWB] 최대 재시도(%d) 초과 — 'range' 입력으로 수동 재시작\n", MAX_RETRY);
        retryCount = 0;
    }
}

// ── UWB 폴링 FSM (loop() 에서 매 틱 호출) ───────────────────
// CLAUDE.md §8: isReceiveDone() 폴링, delay() 금지
static void process_uwb() {
    // 송신 완료 → 수신 모드 전환
    // WAIT_POLL_ACK 상태에서 TX 완료 = POLL TX 확정 → TX_TIME을 즉시 읽어야 정확
    // (이후에 읽으면 send_range()의 DELAYED TX 완료 시 TX_TIME이 덮일 수 있음)
    if (rangeState != RangeState::IDLE && DW1000Ng::isTransmitDone()) {
        if (rangeState == RangeState::WAIT_POLL_ACK) {
            timePollSent = DW1000Ng::getTransmitTimestamp();  // POLL TX 완료 직후 즉시 취득
        }
        DW1000Ng::clearTransmitStatus();
        DW1000Ng::startReceive();
        if (debugMode) Serial.printf("[DBG] TX done → startReceive() at %lu ms\n", millis());
    }

    // PENDING_RANGE_SEND: 대기 시간 경과 → RANGE 전송
    if (rangeState == RangeState::PENDING_RANGE_SEND &&
        (int32_t)(millis() - rangeSendAt) >= 0) {
        rangeState = RangeState::WAIT_RANGE_REPORT;
        deadlineMs = millis() + RANGE_TIMEOUT_MS;
        send_range();
        return;
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
        // timePollSent는 TX 완료 시 이미 취득됨 (POLL TX 직후)
        // timePollAckReceived만 여기서 취득
        timePollAckReceived = DW1000Ng::getReceiveTimestamp();
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive();  // node_car 준비 대기 중에도 RX 유지
        rangeState  = RangeState::PENDING_RANGE_SEND;
        rangeSendAt = millis() + RANGE_SEND_DELAY_MS;
        deadlineMs  = rangeSendAt + RANGE_TIMEOUT_MS;
        Serial.printf("[UWB] POLL_ACK 수신 → %dms 후 RANGE 전송 예약  (rxCount=%lu)\n",
                      RANGE_SEND_DELAY_MS, rxFrameCount);
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

        const float filtered = applyRangeFilter(distanceMeters);
        Serial.printf("[RANGE] Car%u : raw=%.1f cm  filtered=%.1f cm  (%.3f m)\n",
                      carId,
                      distanceMeters * 100.0f,
                      filtered * 100.0f,
                      filtered);
        retryCount = 0;  // 성공 시 재시도 카운터 초기화
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
            retryCount = 0;  // 수동 시작 시 재시도 카운터 초기화
            pendingRetry = false;
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

    // 자동 재시도
    if (pendingRetry && rangeState == RangeState::IDLE &&
        (int32_t)(millis() - retryAt) >= 0) {
        pendingRetry = false;
        start_ranging();
    }

    // UWB FSM 폴링
    process_uwb();
}
