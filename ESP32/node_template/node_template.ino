// ============================================================
//  node_signal.ino  —  신호등 노드 메인
//
//  시리얼 명령 (115200 baud, 시리얼 모니터):
//    RANGE_TRIGGER  → tracking 시작 (1초마다 signal_state 송신)
//    RANGE_STOP     → tracking 종료 (signal_end 1회 후 중단)
//    PED_EXTEND     → 보행 연장 (+3초, 최대 9초)
//    EMERGENCY      → 비상 모드
//    STATUS         → 현재 상태 출력
//
//  ── 업로드 전 확인 ──────────────────────────
//  NODE_ID를 노드 번호에 맞게 수정 (1 / 2 / 3)
// ============================================================

#include "uwb_signal.h"

// ── 노드 ID ──────────────────────────────────
#define NODE_ID  1   // ← 노드 번호에 맞게 수정

// ── LED 핀 ───────────────────────────────────
#define CAR_RED_LED    13
#define CAR_YELLOW_LED 14
#define CAR_GREEN_LED  15
#define PED_RED_LED    32
#define PED_GREEN_LED  33

// ── FSM 타이밍 (ms) ──────────────────────────
#define T_CAR_GREEN   8000
#define T_CAR_YELLOW  2000
#define T_CAR_RED     2000
#define T_ALL_STOP_1  1000
#define T_PED_GREEN   5000
#define T_PED_RED     2000
#define T_ALL_STOP_2  1000
#define T_EMERG_CLEAR  1000
#define T_EMERG_GREEN  10000
#define T_EMERG_RETURN 1000

// ── signal_state 송신 주기 ────────────────────
#define SIGNAL_INTERVAL_MS  1000

// ── 상태 변수 ────────────────────────────────
enum Mode { NORMAL_MODE, EMERGENCY_CLEARANCE, EMERGENCY_GREEN, EMERGENCY_RETURN_CLEARANCE };

Mode          mode          = NORMAL_MODE;
int           phase         = 0;
int           savedPhase    = 0;
unsigned long stateStart    = 0;
unsigned long emergStart    = 0;
unsigned long extraWalk     = 0;   // ms, 최대 9000

bool          tracking      = false;
unsigned long lastSendMs    = 0;

// ── 현재 phase → car/ped 상태값 ──────────────
uint8_t getCarLight() {
    if (phase == 0) return CAR_LIGHT_GREEN;
    if (phase == 1) return CAR_LIGHT_YELLOW;
    return CAR_LIGHT_RED;
}
uint8_t getPedLight() {
    return (phase == 4) ? PED_LIGHT_WALK : PED_LIGHT_RED;
}

// ── 현재 phase 남은 시간(초) ──────────────────
uint8_t getRemainSec() {
    unsigned long total;
    switch (phase) {
        case 0: total = T_CAR_GREEN;                break;
        case 1: total = T_CAR_YELLOW;               break;
        case 2: total = T_CAR_RED;                  break;
        case 3: total = T_ALL_STOP_1;               break;
        case 4: total = T_PED_GREEN + extraWalk;    break;
        case 5: total = T_PED_RED;                  break;
        case 6: total = T_ALL_STOP_2;               break;
        default: total = 0;
    }
    unsigned long elapsed = millis() - stateStart;
    if (elapsed >= total) return 0;
    unsigned long rem = (total - elapsed) / 1000;
    return (rem > 255) ? 255 : (uint8_t)rem;
}

// ── signal_state 송신 ─────────────────────────
void sendSignalState() {
    uint8_t sb = ENCODE_SIGNAL_STATE(
        getCarLight(), getPedLight(),
        extraWalk > 0,
        mode == EMERGENCY_GREEN
    );
    uwb_signal_send_state(sb, getRemainSec());
}

// ── LED 적용 ─────────────────────────────────
void applyPhase() {
    digitalWrite(CAR_RED_LED,    LOW);
    digitalWrite(CAR_YELLOW_LED, LOW);
    digitalWrite(CAR_GREEN_LED,  LOW);
    digitalWrite(PED_RED_LED,    LOW);
    digitalWrite(PED_GREEN_LED,  LOW);

    switch (phase) {
        case 0: digitalWrite(CAR_GREEN_LED,  HIGH); digitalWrite(PED_RED_LED,   HIGH); break;
        case 1: digitalWrite(CAR_YELLOW_LED, HIGH); digitalWrite(PED_RED_LED,   HIGH); break;
        case 2: digitalWrite(CAR_RED_LED,    HIGH); digitalWrite(PED_RED_LED,   HIGH); break;
        case 3: digitalWrite(CAR_RED_LED,    HIGH); digitalWrite(PED_RED_LED,   HIGH); break;
        case 4: digitalWrite(CAR_RED_LED,    HIGH); digitalWrite(PED_GREEN_LED, HIGH); break;
        case 5: digitalWrite(CAR_RED_LED,    HIGH); digitalWrite(PED_RED_LED,   HIGH); break;
        case 6: digitalWrite(CAR_RED_LED,    HIGH); digitalWrite(PED_RED_LED,   HIGH); break;
    }
}

void printPhase() {
    const char* n[] = {"CAR_GREEN","CAR_YELLOW","CAR_RED","ALL_STOP_1","PED_GREEN","PED_RED","ALL_STOP_2"};
    Serial.printf("[PHASE] %s\n", phase <= 6 ? n[phase] : "?");
}

// ── phase 전환 ───────────────────────────────
void nextPhase() {
    phase = (phase + 1) % 7;
    stateStart = millis();
    applyPhase();
    printPhase();
    // tracking 중이면 phase 전환 시 즉시 송신
    if (tracking) {
        sendSignalState();
        lastSendMs = millis();
    }
}

// ── 시리얼 명령 처리 ─────────────────────────
void handleCommand(String cmd) {
    cmd.trim();

    if (cmd == "RANGE_TRIGGER") {
        if (!tracking) {
            tracking = true;
            Serial.println("[TRACK] START");
            sendSignalState();
            lastSendMs = millis();
        } else {
            Serial.println("[TRACK] already tracking");
        }
    }
    else if (cmd == "RANGE_STOP") {
        if (tracking) {
            tracking = false;
            uwb_signal_send_end();
            Serial.println("[TRACK] STOP");
        } else {
            Serial.println("[TRACK] not tracking");
        }
    }
    else if (cmd == "PED_EXTEND") {
        if (phase == 4) {
            if (extraWalk < 9000) {
                extraWalk += 3000;
                Serial.printf("[PED] extended +3s, total extra=%lus\n", extraWalk/1000);
                if (tracking) { sendSignalState(); lastSendMs = millis(); }
            } else {
                Serial.println("[PED] max extension reached");
            }
        } else {
            Serial.println("[PED] ignored: not in PED_GREEN phase");
        }
    }
    else if (cmd == "EMERGENCY") {
        if (mode == NORMAL_MODE) {
            savedPhase = phase;
            extraWalk  = 0;
            mode       = EMERGENCY_CLEARANCE;
            emergStart = millis();
            phase      = 3;
            applyPhase();
            printPhase();
            Serial.println("[MODE] EMERGENCY_CLEARANCE");
            if (tracking) { sendSignalState(); lastSendMs = millis(); }
        }
    }
    else if (cmd == "STATUS") {
        const char* modeStr[] = {"NORMAL","EMERG_CLEAR","EMERG_GREEN","EMERG_RETURN"};
        Serial.printf("phase=%d  mode=%s  extraWalk=%lus  tracking=%s  rem=%ds\n",
            phase, modeStr[mode], extraWalk/1000,
            tracking ? "YES" : "NO", getRemainSec());
    }
    else {
        Serial.println("[CMD] RANGE_TRIGGER / RANGE_STOP / PED_EXTEND / EMERGENCY / STATUS");
    }
}

// ── UWB 수신 처리 ────────────────────────────
void handleUwbRx() {
    char    topic[48];
    uint8_t payload[32];
    uint8_t payloadLen = 0;

    if (!uwb_signal_poll_rx(topic, sizeof(topic), payload, &payloadLen)) return;

    Serial.printf("[UWB RX] topic=%s len=%d\n", topic, payloadLen);

    // 차량이 command/led = 1 보내면 → emergency 처리
    if (strcmp(topic, "command/led") == 0 && payloadLen >= 1) {
        if (payload[0] == 1) handleCommand("EMERGENCY");
    }
}

// ── setup ────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.printf("\n=== node_signal (ID=%d) boot ===\n", NODE_ID);

    pinMode(CAR_RED_LED,    OUTPUT);
    pinMode(CAR_YELLOW_LED, OUTPUT);
    pinMode(CAR_GREEN_LED,  OUTPUT);
    pinMode(PED_RED_LED,    OUTPUT);
    pinMode(PED_GREEN_LED,  OUTPUT);

    phase      = 0;
    stateStart = millis();
    applyPhase();
    printPhase();

    uwb_signal_init(NODE_ID);

    Serial.println("=== Ready ===");
    Serial.println("Commands: RANGE_TRIGGER / RANGE_STOP / PED_EXTEND / EMERGENCY / STATUS");
}

// ── loop ─────────────────────────────────────
void loop() {

    // 1. 시리얼 명령
    if (Serial.available()) {
        handleCommand(Serial.readStringUntil('\n'));
    }

    // 2. UWB 수신
    handleUwbRx();

    // 3. tracking 중: 1초마다 signal_state 송신
    if (tracking && millis() - lastSendMs >= SIGNAL_INTERVAL_MS) {
        sendSignalState();
        lastSendMs = millis();
    }

    // ── Emergency FSM ───────────────────────
    if (mode == EMERGENCY_CLEARANCE) {
        if (millis() - emergStart > T_EMERG_CLEAR) {
            mode = EMERGENCY_GREEN;
            emergStart = millis();
            phase = 0; applyPhase(); printPhase();
            Serial.println("[MODE] EMERGENCY_GREEN");
            if (tracking) { sendSignalState(); lastSendMs = millis(); }
        }
        return;
    }
    if (mode == EMERGENCY_GREEN) {
        if (millis() - emergStart > T_EMERG_GREEN) {
            mode = EMERGENCY_RETURN_CLEARANCE;
            emergStart = millis();
            phase = 6; applyPhase(); printPhase();
            Serial.println("[MODE] RETURN_CLEARANCE");
            if (tracking) { sendSignalState(); lastSendMs = millis(); }
        }
        return;
    }
    if (mode == EMERGENCY_RETURN_CLEARANCE) {
        if (millis() - emergStart > T_EMERG_RETURN) {
            mode = NORMAL_MODE;
            phase = savedPhase;
            stateStart = millis();
            applyPhase(); printPhase();
            Serial.println("[MODE] RETURN_PREVIOUS_PHASE");
            if (tracking) { sendSignalState(); lastSendMs = millis(); }
        }
        return;
    }

    // ── Normal FSM ──────────────────────────
    if      (phase==0 && millis()-stateStart > T_CAR_GREEN)  nextPhase();
    else if (phase==1 && millis()-stateStart > T_CAR_YELLOW) nextPhase();
    else if (phase==2 && millis()-stateStart > T_CAR_RED)    nextPhase();
    else if (phase==3 && millis()-stateStart > T_ALL_STOP_1) nextPhase();
    else if (phase==4) {
        if (millis()-stateStart > T_PED_GREEN + extraWalk) {
            extraWalk = 0;
            nextPhase();
        }
    }
    else if (phase==5 && millis()-stateStart > T_PED_RED)    nextPhase();
    else if (phase==6 && millis()-stateStart > T_ALL_STOP_2) nextPhase();
}
