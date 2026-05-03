// ============================================================
//  node_signal.ino  —  신호등 노드 통합본
//
//  = node_template (UART PING/PONG) + FSM/LED + UWB signal_state
//
//  ── 업로드 전 uart_comm.h 에서 수정 ────────
//    #define NODE_ID  1   (노드 번호 1/2/3)
//
//  ── 시리얼 명령 ─────────────────────────────
//    RANGE_TRIGGER  → UWB tracking 시작
//    RANGE_STOP     → UWB tracking 종료
//    PED_EXTEND     → 보행 연장 (+3초)
//    EMERGENCY      → 비상 모드
//    STATUS         → 현재 상태 출력
//
//  ── RPi CMD_CTRL_FWD 처리 (command/light) ───
//    C0 → 차량 RED (강제)
//    C1 → 차량 GREEN (강제)
//    C9 → EMERGENCY
//    P0 → 보행자 RED (강제)
//    P1 → 보행자 GREEN (PED_EXTEND)
//
//  RPi 입력 예:
//    CMD 1 command/light C9   → Node1만 emergency
//    CMD command/light P1     → 전체 ped extend
// ============================================================

#include "uart_comm.h"
#include "uwb_signal.h"

UartComm uartComm;

// ── LED 핀 ───────────────────────────────────
#define CAR_RED_LED    13
#define CAR_YELLOW_LED 14
#define CAR_GREEN_LED  15
#define PED_RED_LED    32
#define PED_GREEN_LED  33

// ── FSM 타이밍 (ms) ──────────────────────────
#define T_CAR_GREEN    8000
#define T_CAR_YELLOW   2000
#define T_CAR_RED      2000
#define T_ALL_STOP_1   1000
#define T_PED_GREEN    5000
#define T_PED_RED      2000
#define T_ALL_STOP_2   1000
#define T_EMERG_CLEAR  1000
#define T_EMERG_GREEN  10000
#define T_EMERG_RETURN 1000

#define SIGNAL_INTERVAL_MS  1000

// ── 상태 변수 ────────────────────────────────
enum Mode { NORMAL_MODE, EMERGENCY_CLEARANCE, EMERGENCY_GREEN, EMERGENCY_RETURN_CLEARANCE };

Mode          mode       = NORMAL_MODE;
int           phase      = 0;
int           savedPhase = 0;
unsigned long stateStart = 0;
unsigned long emergStart = 0;
unsigned long extraWalk  = 0;

bool          tracking   = false;
unsigned long lastSendMs = 0;

// ── 헬퍼 ─────────────────────────────────────
uint8_t getCarLight() {
    if (phase == 0) return CAR_LIGHT_GREEN;
    if (phase == 1) return CAR_LIGHT_YELLOW;
    return CAR_LIGHT_RED;
}
uint8_t getPedLight() {
    return (phase == 4) ? PED_LIGHT_WALK : PED_LIGHT_RED;
}
uint8_t getRemainSec() {
    unsigned long total;
    switch (phase) {
        case 0: total = T_CAR_GREEN;             break;
        case 1: total = T_CAR_YELLOW;            break;
        case 2: total = T_CAR_RED;               break;
        case 3: total = T_ALL_STOP_1;            break;
        case 4: total = T_PED_GREEN + extraWalk; break;
        case 5: total = T_PED_RED;               break;
        case 6: total = T_ALL_STOP_2;            break;
        default: total = 0;
    }
    unsigned long elapsed = millis() - stateStart;
    if (elapsed >= total) return 0;
    unsigned long rem = (total - elapsed) / 1000;
    return (rem > 255) ? 255 : (uint8_t)rem;
}

void sendSignalState() {
    uint8_t sb = ENCODE_SIGNAL_STATE(
        getCarLight(), getPedLight(),
        extraWalk > 0,
        mode == EMERGENCY_GREEN
    );
    uwb_signal_send_state(sb, getRemainSec());
}

// ── LED ──────────────────────────────────────
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
    if (tracking) { sendSignalState(); lastSendMs = millis(); }
}

// ── 시리얼 명령 ──────────────────────────────
void handleCommand(String cmd) {
    cmd.trim();

    if (cmd == "RANGE_TRIGGER") {
        if (!tracking) {
            tracking = true;
            Serial.println("[TRACK] START");
            sendSignalState();
            lastSendMs = millis();
        } else Serial.println("[TRACK] already tracking");
    }
    else if (cmd == "RANGE_STOP") {
        if (tracking) {
            tracking = false;
            uwb_signal_send_end();
            Serial.println("[TRACK] STOP");
        } else Serial.println("[TRACK] not tracking");
    }
    else if (cmd == "PED_EXTEND") {
        if (phase == 4 && extraWalk < 9000) {
            extraWalk += 3000;
            Serial.printf("[PED] +3s extra=%lus\n", extraWalk/1000);
            if (tracking) { sendSignalState(); lastSendMs = millis(); }
        } else Serial.println("[PED] ignored");
    }
    else if (cmd == "EMERGENCY") {
        if (mode == NORMAL_MODE) {
            savedPhase = phase; extraWalk = 0;
            mode = EMERGENCY_CLEARANCE;
            emergStart = millis();
            phase = 3; applyPhase(); printPhase();
            Serial.println("[MODE] EMERGENCY_CLEARANCE");
            if (tracking) { sendSignalState(); lastSendMs = millis(); }
        }
    }
    else if (cmd == "STATUS") {
        const char* ms[] = {"NORMAL","EMERG_CLEAR","EMERG_GREEN","EMERG_RETURN"};
        Serial.printf("phase=%d mode=%s extraWalk=%lus tracking=%s rem=%ds\n",
            phase, ms[mode], extraWalk/1000, tracking?"YES":"NO", getRemainSec());
    }
    else Serial.println("[CMD] RANGE_TRIGGER/RANGE_STOP/PED_EXTEND/EMERGENCY/STATUS");
}

// ── RPi CMD_CTRL_FWD 콜백 ────────────────────
// command/light payload:
//   C0 → 차량 RED   C1 → 차량 GREEN   C9 → EMERGENCY
//   P0 → 보행 RED   P1 → 보행 GREEN (PED_EXTEND)
void onCtrlFwd(const char* topic, const uint8_t* payload, uint8_t len) {
    if (strcmp(topic, "command/light") == 0 && len >= 2) {
        char code[3] = {0};
        code[0] = (char)payload[0];
        code[1] = (char)payload[1];

        Serial.printf("[LIGHT] cmd=%c%c\n", code[0], code[1]);

        if (code[0] == 'C') {
            switch (code[1]) {
                case '9': handleCommand("EMERGENCY");    break;
                case '1':
                    // 차량 GREEN 강제: CAR_GREEN phase로
                    if (mode == NORMAL_MODE) {
                        phase = 0; stateStart = millis();
                        applyPhase(); printPhase();
                        if (tracking) { sendSignalState(); lastSendMs = millis(); }
                    }
                    break;
                case '0':
                    // 차량 RED 강제: CAR_RED phase로
                    if (mode == NORMAL_MODE) {
                        phase = 2; stateStart = millis();
                        applyPhase(); printPhase();
                        if (tracking) { sendSignalState(); lastSendMs = millis(); }
                    }
                    break;
                default:
                    Serial.printf("[LIGHT] 알 수 없는 차량 코드: C%c\n", code[1]);
            }
        }
        else if (code[0] == 'P') {
            switch (code[1]) {
                case '1': handleCommand("PED_EXTEND"); break;
                case '0':
                    // 보행자 RED 강제: extraWalk 초기화
                    extraWalk = 0;
                    Serial.println("[LIGHT] PED_RED 강제");
                    if (tracking) { sendSignalState(); lastSendMs = millis(); }
                    break;
                default:
                    Serial.printf("[LIGHT] 알 수 없는 보행 코드: P%c\n", code[1]);
            }
        }
        else {
            Serial.printf("[LIGHT] 알 수 없는 prefix: %c\n", code[0]);
        }
    }
    // 하위 호환: range trigger/stop도 RPi에서 제어 가능
    else if (strcmp(topic, "command/range_trigger") == 0) {
        handleCommand("RANGE_TRIGGER");
    }
    else if (strcmp(topic, "command/range_stop") == 0) {
        handleCommand("RANGE_STOP");
    }
}

// ── UWB 수신 ─────────────────────────────────
void handleUwbRx() {
    char    topic[48];
    uint8_t payload[32];
    uint8_t payloadLen = 0;
    if (!uwb_signal_poll_rx(topic, sizeof(topic), payload, &payloadLen)) return;
    Serial.printf("[UWB RX] topic=%s len=%d\n", topic, payloadLen);
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

    phase = 0; stateStart = millis();
    applyPhase(); printPhase();

    uartComm.setCtrlFwdCallback(onCtrlFwd);
    uartComm.begin();

    uwb_signal_init(NODE_ID);

    Serial.println("=== Ready ===");
}

// ── loop ─────────────────────────────────────
void loop() {
    // 1. UART (RPi 통신)
    uartComm.process();

    // 2. 시리얼 명령
    if (Serial.available()) {
        handleCommand(Serial.readStringUntil('\n'));
    }

    // 3. UWB 수신
    handleUwbRx();

    // 4. tracking 중 1초마다 signal_state 송신
    if (tracking && millis() - lastSendMs >= SIGNAL_INTERVAL_MS) {
        sendSignalState();
        lastSendMs = millis();
    }

    // ── Emergency FSM ───────────────────────
    if (mode == EMERGENCY_CLEARANCE) {
        if (millis() - emergStart > T_EMERG_CLEAR) {
            mode = EMERGENCY_GREEN; emergStart = millis();
            phase = 0; applyPhase(); printPhase();
            Serial.println("[MODE] EMERGENCY_GREEN");
            if (tracking) { sendSignalState(); lastSendMs = millis(); }
        }
        return;
    }
    if (mode == EMERGENCY_GREEN) {
        if (millis() - emergStart > T_EMERG_GREEN) {
            mode = EMERGENCY_RETURN_CLEARANCE; emergStart = millis();
            phase = 6; applyPhase(); printPhase();
            Serial.println("[MODE] RETURN_CLEARANCE");
            if (tracking) { sendSignalState(); lastSendMs = millis(); }
        }
        return;
    }
    if (mode == EMERGENCY_RETURN_CLEARANCE) {
        if (millis() - emergStart > T_EMERG_RETURN) {
            mode = NORMAL_MODE;
            phase = savedPhase; stateStart = millis();
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
            extraWalk = 0; nextPhase();
        }
    }
    else if (phase==5 && millis()-stateStart > T_PED_RED)    nextPhase();
    else if (phase==6 && millis()-stateStart > T_ALL_STOP_2) nextPhase();
}
