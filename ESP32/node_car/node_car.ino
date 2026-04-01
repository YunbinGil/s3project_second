// node_car.ino — 차량 노드 (DWM1000 UWB + TB6612FNG 모터 + WiFi AP)
//
// motor.ino 기반, 아래 버그 수정 적용:
//   [fix-1] sendPollAckFrame / sendRangeReportFrame / sendRangeFailedFrame / sendAck
//            : delay() 제거 → millis() 기반 100ms 타임아웃
//   [fix-2] 모든 TX 함수: setTransmitData 호출 전 forceTRxOff() 추가
//            (RX 상태에서 TX 전환 시 isTransmitDone()이 영구 false 되는 문제 해결)
//   [fix-3] sendPollAckFrame / sendRangeReportFrame / sendRangeFailedFrame
//            : TX 완료 후 startReceive() 추가 (CLAUDE.md §8 절대 금지 준수)
//
// ── 핀 배치 (CLAUDE.md §4.3) ────────────────────────────────
//   DWM1000 : SCK=18, MISO=19, MOSI=23, SS=5, RST=22
//   TB6612  : PWMA=15, AIN1=27, AIN2=13, STBY=14

#include <WiFi.h>
#include <WebServer.h>
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgConstants.hpp>
#include <SPI.h>

// ── Wi-Fi 설정 ────────────────────────────────────────────────
#define WIFI_SSID     "ESP32"
#define WIFI_PASSWORD "12345678"

// ── DWM1000 핀 ───────────────────────────────────────────────
const uint8_t PIN_SCK  = 18;
const uint8_t PIN_MISO = 19;
const uint8_t PIN_MOSI = 23;
const uint8_t PIN_SS   = 5;
const uint8_t PIN_RST  = 22;
const uint8_t PIN_IRQ  = 4;

// ── 모터 핀 ──────────────────────────────────────────────────
#define PIN_PWMA  15
#define PIN_AIN1  27
#define PIN_AIN2  13
#define PIN_STBY  14

#define PWM_FREQ  20000
#define PWM_UNIT  MCPWM_UNIT_0
#define PWM_TIMER MCPWM_TIMER_0

// ── UWB 프레임 타입 ───────────────────────────────────────────
enum UwbRangeMessageType : uint8_t {
    UWB_POLL         = 0,
    UWB_POLL_ACK     = 1,
    UWB_RANGE        = 2,
    UWB_RANGE_REPORT = 3,
    UWB_RANGE_FAILED = 255,
    UWB_MODE_NOTIFY  = 0xF1
};

// ── Proto-B (MQTT-over-UWB) 상수 ─────────────────────────────
enum MessageType : uint8_t { PUBLISH = 0x01, SUBSCRIBE = 0x02, ACK = 0x03, PING = 0x04, PONG = 0x05 };
enum QoS         : uint8_t { AT_MOST_ONCE = 0, AT_LEAST_ONCE = 1, EXACTLY_ONCE = 2 };

const uint8_t START_BYTE = 0x7E;
const uint8_t END_BYTE   = 0x7F;

// ── FSM 제어 모드 (CLAUDE.md §10) ────────────────────────────
enum ControlMode : uint8_t { HTTP_CONTROL = 0, MQTT_CONTROL = 1 };

// ── 전역 상수 ─────────────────────────────────────────────────
const uint8_t CAR_ID = 6;
const int UWB_RANGE_FRAME_LEN = 16;

device_configuration_t DEFAULT_CONFIG = {
    false, true, true, true, false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

// ── 전역 변수 ─────────────────────────────────────────────────
WebServer server(80);
float currentSpeed = 0.0f;
float targetSpeed  = 0.0f;
const float acceleration = 2.5f;
bool  currentDirection   = true;
ControlMode controlMode  = HTTP_CONTROL;

uint16_t nextMessageId = 1;
uint8_t  recvBuffer[1024];
uint8_t  txBuffer[256];
int16_t  numReceived = 0;

byte     rangeFrame[UWB_RANGE_FRAME_LEN] = {0};
uint64_t timePollReceived  = 0;
uint64_t timePollAckSent   = 0;
bool     rangingProtocolFailed    = false;
float    lastComputedRangeMeters  = -1.0f;

bool     pendingModeNotify = false;
uint8_t  pendingMode       = 0;

String subscribedTopics[10];
int    subscribedCount = 0;

unsigned long lastMotorUpdate = 0;

// (진단용) RANGE 대기 상태 추적
static bool waitingForRange = false;

// ── 함수 원형 선언 ────────────────────────────────────────────
uint16_t calculateCRC16(uint8_t* data, int len);
bool     parseMessage(uint8_t* data, int len);
void     handlePublish(uint16_t msgId, uint8_t qos, String topic, uint8_t* payload, int len);
void     sendAck(uint16_t msgId);
bool     handleRangingFrame(uint8_t* data, int len, uint64_t rxTimestamp);
void     sendPollAckFrame();
void     sendRangeReportFrame(float meters);
void     sendRangeFailedFrame();
void     sendModeNotifyFrame(uint8_t mode);
void     setTargetSpeed(float speed, ControlMode requiredMode);

// ============================================================
// 모터 제어 (CLAUDE.md §11)
// ============================================================
void initMotor() {
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_AIN1) | (1ULL << PIN_AIN2) | (1ULL << PIN_STBY);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level((gpio_num_t)PIN_STBY, 1);

    mcpwm_gpio_init(PWM_UNIT, MCPWM0A, PIN_PWMA);

    mcpwm_config_t pwm_config;
    pwm_config.frequency    = PWM_FREQ * 2;
    pwm_config.cmpr_a       = 0;
    pwm_config.cmpr_b       = 0;
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;
    pwm_config.duty_mode    = MCPWM_DUTY_MODE_0;
    mcpwm_init(PWM_UNIT, PWM_TIMER, &pwm_config);
}

// CLAUDE.md §10: targetSpeed 는 반드시 이 함수를 경유
void setTargetSpeed(float speed, ControlMode requiredMode) {
    if (controlMode == requiredMode) {
        targetSpeed = speed;
        Serial.printf("[Speed] Set to %.1f (Mode: %s)\n", speed,
                      requiredMode == HTTP_CONTROL ? "HTTP" : "MQTT");
    } else {
        Serial.printf("[Speed] IGNORED — Current: %s, Req: %s\n",
                      controlMode == HTTP_CONTROL ? "HTTP" : "MQTT",
                      requiredMode == HTTP_CONTROL ? "HTTP" : "MQTT");
    }
}

void setMotor(float speed, bool direction) {
    gpio_set_level((gpio_num_t)PIN_AIN1, direction ? 1 : 0);
    gpio_set_level((gpio_num_t)PIN_AIN2, direction ? 0 : 1);
    mcpwm_set_duty(PWM_UNIT, PWM_TIMER, MCPWM_OPR_A, speed);
    mcpwm_set_duty_type(PWM_UNIT, PWM_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

void updateMotorSpeed(bool direction) {
    if (millis() - lastMotorUpdate < 20) return;
    lastMotorUpdate = millis();

    if (abs(currentSpeed - targetSpeed) > 0.1f) {
        if (currentSpeed < targetSpeed) {
            currentSpeed += acceleration;
            if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
        } else {
            currentSpeed -= acceleration;
            if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
        }
        setMotor(abs(currentSpeed), direction);
    }
}

// ============================================================
// WebServer (CLAUDE.md §12)
// ============================================================
void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>body{font-family:sans-serif;text-align:center;margin-top:50px;background:#222;color:white;}";
    html += ".btn{padding:20px 40px;font-size:24px;margin:10px;border:none;border-radius:10px;color:white;cursor:pointer;width:80%;max-width:300px;}";
    html += ".btn-up{background:#4CAF50;}.btn-down{background:#f44336;}</style></head><body>";
    html += "<h1>ESP32 Motor Control</h1>";
    html += "<p>Current Speed: <span id='s'>" + String((int)currentSpeed) + "</span>%</p>";
    html += "<button class='btn btn-up' onclick=\"fetch('/up').then(r=>r.text()).then(d=>document.getElementById('s').innerText=d)\">Speed UP (+10)</button><br>";
    html += "<button class='btn btn-down' onclick=\"fetch('/down').then(r=>r.text()).then(d=>document.getElementById('s').innerText=d)\">Speed DOWN (-10)</button>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void handleSpeedUp() {
    // CLAUDE.md §12: controlMode 검사 필수
    if (controlMode != HTTP_CONTROL) {
        server.send(403, "text/plain", "MQTT Control Mode Active");
        Serial.println("[HTTP] Speed UP BLOCKED — MQTT mode active");
        return;
    }
    float newTarget = targetSpeed + 10.0f;
    if (newTarget > 100.0f) newTarget = 100.0f;
    setTargetSpeed(newTarget, HTTP_CONTROL);
    server.send(200, "text/plain", String((int)targetSpeed));
}

void handleSpeedDown() {
    if (controlMode != HTTP_CONTROL) {
        server.send(403, "text/plain", "MQTT Control Mode Active");
        Serial.println("[HTTP] Speed DOWN BLOCKED — MQTT mode active");
        return;
    }
    float newTarget = targetSpeed - 10.0f;
    if (newTarget < 0.0f) newTarget = 0.0f;
    setTargetSpeed(newTarget, HTTP_CONTROL);
    server.send(200, "text/plain", String((int)targetSpeed));
}

// ============================================================
// MQTT-over-UWB (Proto-B) 구독/발행
// ============================================================
void subscribe(String topic) {
    if (subscribedCount < 10) subscribedTopics[subscribedCount++] = topic;
}

bool isSubscribed(String topic) {
    for (int i = 0; i < subscribedCount; i++) {
        if (subscribedTopics[i] == topic) return true;
    }
    return false;
}

uint16_t calculateCRC16(uint8_t* data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc = crc << 1;
        }
    }
    return crc;
}

void publish(String topic, uint8_t* payload, int payloadLen) {
    uint16_t msgId = nextMessageId++;
    if (nextMessageId == 0) nextMessageId = 1;

    int bufferSize = 11 + (int)topic.length() + payloadLen;
    if (bufferSize > (int)sizeof(txBuffer)) {
        Serial.println("[Error] Tx Buffer overflow");
        return;
    }

    int pos = 0;
    txBuffer[pos++] = START_BYTE;
    txBuffer[pos++] = PUBLISH;
    txBuffer[pos++] = (msgId >> 8) & 0xFF;
    txBuffer[pos++] =  msgId       & 0xFF;
    txBuffer[pos++] = AT_LEAST_ONCE;
    txBuffer[pos++] = (uint8_t)topic.length();
    for (int i = 0; i < (int)topic.length(); i++) txBuffer[pos++] = (uint8_t)topic[i];
    txBuffer[pos++] = (payloadLen >> 8) & 0xFF;
    txBuffer[pos++] =  payloadLen       & 0xFF;
    for (int i = 0; i < payloadLen; i++) txBuffer[pos++] = payload[i];

    uint16_t crc = calculateCRC16(txBuffer + 1, pos - 1);
    txBuffer[pos++] = (crc >> 8) & 0xFF;
    txBuffer[pos++] =  crc       & 0xFF;
    txBuffer[pos++] = END_BYTE;

    // [fix-2] forceTRxOff: RX 상태에서 TX 전환 안정화
    DW1000Ng::forceTRxOff();
    DW1000Ng::setTransmitData(txBuffer, pos);
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);
    const uint32_t deadline = millis() + 100;
    while (!DW1000Ng::isTransmitDone()) {
        if ((int32_t)(millis() - deadline) > 0) {
            Serial.println("[UWB] publish 전송 타임아웃");
            break;
        }
    }
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::startReceive();  // publish TX 완료 후 RX 복귀
}

void publish(String topic, String payload) {
    publish(topic, (uint8_t*)payload.c_str(), (int)payload.length());
}

// ============================================================
// UWB TWR 프레임 전송 — 수정 포인트 3종 적용
// ============================================================

// POLL_ACK 전송
// 폴링 방식이므로 setTransmitData 전에 forceTRxOff() 필수
// (인터럽트 방식 예제와 달리 폴링 방식에서의 RX→TX 전환 안정화)
void sendPollAckFrame() {
    memset(rangeFrame, 0, sizeof(rangeFrame));
    rangeFrame[0] = UWB_POLL_ACK;

    DW1000Ng::forceTRxOff();                                 // RX→TX 전환 안정화
    DW1000Ng::setTransmitData(rangeFrame, UWB_RANGE_FRAME_LEN);
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    const uint32_t deadline = millis() + 100;                // [fix-1] delay() 없음
    while (!DW1000Ng::isTransmitDone()) {
        if ((int32_t)(millis() - deadline) > 0) {
            Serial.println("[UWB] POLL_ACK 전송 타임아웃");
            DW1000Ng::forceTRxOff();
            DW1000Ng::startReceive();
            rangingProtocolFailed = true;
            return;
        }
    }
    timePollAckSent = DW1000Ng::getTransmitTimestamp();     // clearTransmitStatus() 전에 취득
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::startReceive();                                // [fix-3] 반이중 수신 복귀
}

// [fix-1][fix-2][fix-3] RANGE_REPORT 전송
void sendRangeReportFrame(float meters) {
    memset(rangeFrame, 0, sizeof(rangeFrame));
    rangeFrame[0] = UWB_RANGE_REPORT;
    float encodedDistance = meters * DISTANCE_OF_RADIO_INV;
    memcpy(rangeFrame + 1, &encodedDistance, sizeof(float));
    rangeFrame[5] = CAR_ID;

    DW1000Ng::forceTRxOff();                                 // [fix-2]
    DW1000Ng::setTransmitData(rangeFrame, UWB_RANGE_FRAME_LEN);
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    const uint32_t deadline = millis() + 100;                // [fix-1]
    while (!DW1000Ng::isTransmitDone()) {
        if ((int32_t)(millis() - deadline) > 0) {
            Serial.println("[UWB] RANGE_REPORT 전송 타임아웃");
            break;
        }
    }
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::startReceive();                                // [fix-3]
}

void sendRangeFailedFrame() {
    memset(rangeFrame, 0, sizeof(rangeFrame));
    rangeFrame[0] = UWB_RANGE_FAILED;

    DW1000Ng::forceTRxOff();                                 // [fix-2]
    DW1000Ng::setTransmitData(rangeFrame, UWB_RANGE_FRAME_LEN);
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    const uint32_t deadline = millis() + 100;                // [fix-1]
    while (!DW1000Ng::isTransmitDone()) {
        if ((int32_t)(millis() - deadline) > 0) {
            Serial.println("[UWB] RANGE_FAILED 전송 타임아웃");
            break;
        }
    }
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::startReceive();                                // [fix-3]
}

// [fix-1][fix-2] ACK 전송 (MQTT QoS AT_LEAST_ONCE 응답)
void sendAck(uint16_t msgId) {
    uint8_t buffer[11];
    int pos = 0;
    buffer[pos++] = START_BYTE;
    buffer[pos++] = ACK;
    buffer[pos++] = (msgId >> 8) & 0xFF;
    buffer[pos++] =  msgId       & 0xFF;
    buffer[pos++] = AT_MOST_ONCE;
    buffer[pos++] = 0;   // topic len
    buffer[pos++] = 0;   // payload len hi
    buffer[pos++] = 0;   // payload len lo
    uint16_t crc = calculateCRC16(buffer + 1, pos - 1);
    buffer[pos++] = (crc >> 8) & 0xFF;
    buffer[pos++] =  crc       & 0xFF;
    buffer[pos++] = END_BYTE;

    DW1000Ng::forceTRxOff();                                 // [fix-2]
    DW1000Ng::setTransmitData(buffer, pos);
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    const uint32_t deadline = millis() + 100;                // [fix-1]
    while (!DW1000Ng::isTransmitDone()) {
        if ((int32_t)(millis() - deadline) > 0) {
            Serial.println("[UWB] ACK 전송 타임아웃");
            break;
        }
    }
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::startReceive();  // ACK TX 완료 후 RX 복귀
}

// [fix-2] MODE_NOTIFY 전송
void sendModeNotifyFrame(uint8_t mode) {
    byte notifyFrame[UWB_RANGE_FRAME_LEN] = {0};
    notifyFrame[0] = (byte)UWB_MODE_NOTIFY;
    notifyFrame[1] = CAR_ID;
    notifyFrame[2] = mode;

    DW1000Ng::forceTRxOff();                                 // [fix-2]
    DW1000Ng::setTransmitData(notifyFrame, UWB_RANGE_FRAME_LEN);
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    const uint32_t deadline = millis() + 100;
    while (!DW1000Ng::isTransmitDone()) {
        if ((int32_t)(millis() - deadline) > 0) {
            Serial.println("[UWB] MODE_NOTIFY 전송 타임아웃");
            break;
        }
    }
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::startReceive();
    Serial.printf("[UWB] MODE_NOTIFY 전송: car=%u mode=%u\n", CAR_ID, mode);
}

// ============================================================
// UWB 수신 처리 (motor.ino processMessages 구조 동일)
// ============================================================
// rxTimestamp: processMessages에서 clearReceiveStatus 전에 이미 취득한 값
bool handleRangingFrame(uint8_t* data, int len, uint64_t rxTimestamp) {
    if (len < 1) return false;
    
    byte msgId = data[0];
    if (msgId == UWB_POLL || msgId == UWB_RANGE) {
        // 정확히 일치(==) 대신, 최소 길이(>=)만 만족하면 통과시킴 (CRC 바이트 무시)
        if (len < UWB_RANGE_FRAME_LEN) {
            Serial.println("[UWB] 에러: TWR 프레임 길이가 너무 짧습니다.");
            return false;
        }
        
        if (msgId == UWB_POLL) {
            rangingProtocolFailed = false;
            timePollReceived = rxTimestamp;
            // Serial.println("[UWB] POLL 수신 → POLL_ACK 전송 중...");
            sendPollAckFrame();
            // Serial.println("[UWB] POLL_ACK 전송 완료 → RANGE 대기");
            waitingForRange = true;
            return true;
        }

        // UWB_RANGE 수신 처리
        uint64_t timeRangeReceived = rxTimestamp;
        waitingForRange = false;
        Serial.println("[UWB] RANGE 수신 → 거리 계산 중...");
        if (!rangingProtocolFailed) {
            uint64_t timePollSent        = DW1000NgUtils::bytesAsValue(data + 1,  LENGTH_TIMESTAMP);
            uint64_t timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 6,  LENGTH_TIMESTAMP);
            uint64_t timeRangeSent       = DW1000NgUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);

            double distance = DW1000NgRanging::computeRangeAsymmetric(
                timePollSent, timePollReceived,
                timePollAckSent, timePollAckReceived,
                timeRangeSent, timeRangeReceived);
            distance = DW1000NgRanging::correctRange(distance);
            Serial.printf("[UWB] 계산된 거리: %.3f m\n", distance);

            if (distance <= 0.0 || distance > 100.0) {
                Serial.printf("[UWB] 범위 밖 (%.3f m) → RANGE_FAILED 전송\n", distance);
                sendRangeFailedFrame();
            } else {
                Serial.printf("[UWB] 유효 거리 → RANGE_REPORT 전송: %.3f m\n", distance);
                sendRangeReportFrame((float)distance);
                lastComputedRangeMeters = (float)distance;
                publish("telemetry/range/last", String((float)distance, 3));
            }
        } else {
            Serial.println("[UWB] rangingProtocolFailed=true → RANGE_FAILED 전송");
            sendRangeFailedFrame();
        }
        return true;
    }
    return false;
}

bool parseMessage(uint8_t* data, int len) {

    if (len < 11) return false; // 최소 헤더 길이 검사

    int pos = 1;
    uint8_t  msgType   = data[pos++];
    uint16_t msgId     = ((uint16_t)data[pos] << 8) | data[pos + 1]; pos += 2;
    uint8_t  qos       = data[pos++];
    uint8_t  topicLen  = data[pos++];

    if (pos + topicLen + 2 > len) return false;

    String topic = "";
    for (int i = 0; i < topicLen; i++) topic += (char)data[pos++];

    uint16_t payloadLen = ((uint16_t)data[pos] << 8) | data[pos + 1]; pos += 2;
    
    int expectedLen = pos + payloadLen + 3;

    if (expectedLen > len) {
        Serial.println("[Error] Packet too short for parsed length");
        return false; 
    }

    // 진짜 길이가 끝나는 지점에 END_BYTE가 있는지 확인
    if (data[expectedLen - 1] != END_BYTE) {
        Serial.println("[Error] END_BYTE mismatch");
        return false;
    }

    uint8_t* payload = data + pos;
    pos += payloadLen;

    uint16_t receivedCRC   = ((uint16_t)data[pos] << 8) | data[pos + 1];
    uint16_t calculatedCRC = calculateCRC16(data + 1, pos - 1);
    
    if (receivedCRC != calculatedCRC) {
        Serial.println("[Error] CRC mismatch");
        return false;
    }

    if      (msgType == PUBLISH) handlePublish(msgId, qos, topic, payload, (int)payloadLen);
    else if (msgType == ACK)     Serial.printf("[ACK] msgId=%u\n", msgId);
    return true;
}

void handlePublish(uint16_t msgId, uint8_t qos, String topic, uint8_t* payload, int len) {
    if (!isSubscribed(topic)) return;
    if (qos == AT_LEAST_ONCE) sendAck(msgId);

    if (topic == "command/controlled") {
        if (len > 0) {
            if (payload[0] == '1' || payload[0] == 1) {
                controlMode = MQTT_CONTROL;
                Serial.println("[FSM] Mode: MQTT_CONTROL (HTTP BLOCKED)");
            } else {
                controlMode = HTTP_CONTROL;
                Serial.println("[FSM] Mode: HTTP_CONTROL (HTTP ENABLED)");
            }
            // processMessages() 완료 후 loop()에서 UWB 알림 전송 (즉시 전송 금지)
            pendingModeNotify = true;
            pendingMode = (uint8_t)controlMode;
        }
    } else if (topic == "command/led") {
        if (len > 0) {
            bool on = (payload[0] == '1' || payload[0] == 1);
            digitalWrite(2, on ? HIGH : LOW);
            Serial.printf("[CMD] LED %s\n", on ? "ON" : "OFF");
        }
    } else if (topic == "command/motor") {
        if (len > 0) {
            char buf[32];
            int copyLen = (len < 31) ? len : 31;
            memcpy(buf, payload, copyLen);
            buf[copyLen] = '\0';
            float speed = atof(buf);
            if (speed < 0.0f)   speed = 0.0f;
            if (speed > 100.0f) speed = 100.0f;
            setTargetSpeed(speed, MQTT_CONTROL);
        }
    } else if (topic == "command/range_probe") {
        if (lastComputedRangeMeters >= 0.0f)
            publish("telemetry/range/last", String(lastComputedRangeMeters, 3));
        else
            publish("telemetry/range/last", "-1");
    } else {
        Serial.printf("[Recv] Topic: %s  PayloadLen: %d\n", topic.c_str(), len);
    }
}

void processMessages() {
    
    if (!DW1000Ng::isReceiveDone()) return;

    int dataLen = DW1000Ng::getReceivedDataLength();
    numReceived++;

    // ── 진단: 수신된 모든 프레임 덤프 ──────────────────────────
    Serial.printf("[RX#%d] len=%d  data: ", numReceived, dataLen);
    // 내용은 아래서 읽은 후 출력하므로 일단 len만

    if (dataLen > (int)sizeof(recvBuffer) || dataLen < 1) {
        Serial.printf("← INVALID SIZE\n");
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive();
        return;
    }

    byte tempBuffer[dataLen];
    DW1000Ng::getReceivedData(tempBuffer, dataLen);

    uint64_t rxTimestamp = DW1000Ng::getReceiveTimestamp();  // (1)
    DW1000Ng::clearReceiveStatus();                          // (2)

    // 덤프 출력
    for (int i = 0; i < dataLen && i < 16; i++) Serial.printf("%02X ", tempBuffer[i]);
    if (dataLen > 16) Serial.print("...");
    Serial.println();

    bool handledAsRanging = handleRangingFrame(tempBuffer, dataLen, rxTimestamp);

    if (!handledAsRanging) {
        if (tempBuffer[0] == START_BYTE) {
            if (parseMessage(tempBuffer, dataLen)) {
                // Serial.println("[UWB] MQTT Message Parsed Successfully.");
            } else {
                Serial.println("[UWB] MQTT Parsing Failed (CRC or Format Error).");
            }
        }
        DW1000Ng::startReceive();
    }
    // Ranging 경로: sendPollAckFrame / sendRangeReportFrame / sendRangeFailedFrame
    // 내부에서 이미 startReceive() 호출됨
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.println(F("=== node_car 초기화 ==="));

    // DWM1000 초기화 (CLAUDE.md §8)
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::setDeviceAddress(CAR_ID);
    DW1000Ng::setNetworkId(10);
    DW1000Ng::setAntennaDelay(16436);

    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network & Addr: "); Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);

    // 모터 초기화
    initMotor();
    setMotor(0, true);

    // WiFi AP 설정 (CLAUDE.md §12)
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

    // WebServer
    server.on("/",     handleRoot);
    server.on("/up",   handleSpeedUp);
    server.on("/down", handleSpeedDown);
    server.begin();
    Serial.println("Web server started");

    // 토픽 구독
    subscribe("command/controlled");
    subscribe("command/motor");
    subscribe("command/led");
    subscribe("command/range_probe");
    subscribe("status/ranging");

    // UWB 수신 시작 (CLAUDE.md §8)
    DW1000Ng::startReceive();
    Serial.println("DW1000 Receive mode started");
    Serial.println("=== 초기화 완료 ===");
}

// CLAUDE.md §4.3 loop() 구조
void loop() {
    // 진단: loop() 생존 확인 + isReceiveDone 상태 (1초 주기)
    server.handleClient();       // HTTP 클라이언트 처리

    processMessages();           // UWB 수신 처리

    // processMessages() 완료 후 UWB MODE_NOTIFY 전송 (수신 버퍼 오염 방지)
    if (pendingModeNotify) {
        pendingModeNotify = false;
        sendModeNotifyFrame(pendingMode);
    }

    updateMotorSpeed(currentDirection);  // 모터 가감속
    // (waitingForRange 중 강제 RX 재초기화 제거 — RANGE 도달 타이밍 방해됨)
}
