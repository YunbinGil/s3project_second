#include <WiFi.h>
#include <WebServer.h>
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include <HardwareSerial.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgConstants.hpp>
#include <SPI.h>


// ==========================================
// 1. Wi-Fi 설정 
// ==========================================
const char* ssid = "ESP32";      // 와이파이 이름
const char* password = "12345678"; // 와이파이 비밀번호

// ==========================================
// 2. 핀 및 상수 정의 (제공해주신 코드 기반)
// ==========================================

const uint8_t PIN_SCK = 18;
const uint8_t PIN_MISO = 19;
const uint8_t PIN_MOSI = 23;
const uint8_t PIN_SS = 5;
const uint8_t PIN_RST = 22; 
const uint8_t PIN_IRQ = 4;

#define PIN_PWMA 15
#define PIN_AIN1 27
#define PIN_AIN2 13
#define PIN_STBY 14

#define PWM_FREQ 20000
#define PWM_UNIT MCPWM_UNIT_0
#define PWM_TIMER MCPWM_TIMER_0

// 메시지 타입
enum MessageType {
  PUBLISH = 0x01,
  SUBSCRIBE = 0x02,
  ACK = 0x03,
  PING = 0x04,
  PONG = 0x05
};

enum UwbRangeMessageType {
  UWB_POLL = 0,
  UWB_POLL_ACK = 1,
  UWB_RANGE = 2,
  UWB_RANGE_REPORT = 3,
  UWB_RANGE_FAILED = 255,
  UWB_MODE_NOTIFY = 0xF1   // motor → 신호등: FSM 모드 전환 알림
};

// QoS 레벨
enum QoS {
  AT_MOST_ONCE = 0,
  AT_LEAST_ONCE = 1,
  EXACTLY_ONCE = 2
};

uint8_t CAR_ID = 6;
//DWM RECV/SEND 정보
device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,     // 2번 대신 기본 5번 사용
    DataRate::RATE_850KBPS, // 110KBPS -> 6.8MBPS (고속 모드가 설정 오류가 적음)
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256, // 고속 모드에 맞는 짧은 프리앰블
    PreambleCode::CODE_3
};



// FSM: 제어 모드 정의
enum ControlMode {
  HTTP_CONTROL = 0,  // HTTP(웹)에서 제어
  MQTT_CONTROL = 1   // MQTT(DWM)에서 제어
};

WebServer server(80);
float currentSpeed = 0.0; // 현재 속도 (0.0 ~ 100.0)
float targetSpeed = 0.0;
const float acceleration = 2.5; // 속도 변화량

bool currentDirection = true; // 방향 (true: 정회전, false: 역회전)
ControlMode controlMode = HTTP_CONTROL; // 초기 상태: HTTP 제어

const uint8_t START_BYTE = 0x7E;
const uint8_t END_BYTE = 0x7F;

uint16_t nextMessageId = 1;

uint8_t recvBuffer[1024]; // ESP32의 넉넉한 RAM 활용
int recvBufferLen = 0;

String subscribedTopics[10];
int subscribedCount = 0;

unsigned long lastPublish = 0;

uint8_t txBuffer[256];

int16_t numReceived = 0;
String message;

const int UWB_RANGE_FRAME_LEN = 16;
byte rangeFrame[UWB_RANGE_FRAME_LEN] = {0};
uint64_t timePollSent = 0;
uint64_t timePollReceived = 0;
uint64_t timePollAckSent = 0;
uint64_t timePollAckReceived = 0;
uint64_t timeRangeSent = 0;
uint64_t timeRangeReceived = 0;
bool rangingProtocolFailed = false;
float lastComputedRangeMeters = -1.0;
bool pendingModeNotify = false;   // loop()에서 MODE_NOTIFY 전송 트리거
uint8_t pendingMode = 0;


// ========== 함수 원형 선언 ==========
uint16_t calculateCRC16(uint8_t* data, int len);
bool parseMessage(uint8_t* data, int len);
void handlePublish(uint16_t msgId, uint8_t qos, String topic, uint8_t* payload, int len);
void handleAck(uint16_t msgId);
void sendAck(uint16_t msgId);
bool handleRangingFrame(uint8_t* data, int len);
void sendPollAckFrame();
void sendRangeReportFrame(float meters);
void sendRangeFailedFrame();
void sendModeNotifyFrame(uint8_t mode);

// ==========================================
// 4. 모터 제어 함수 (제공해주신 코드)
// ==========================================
void initMotor() {
  // GPIO 초기화
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = ((1ULL << PIN_AIN1) | (1ULL << PIN_AIN2) | (1ULL << PIN_STBY));
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  // STBY High
  gpio_set_level((gpio_num_t)PIN_STBY, 1);

  // MCPWM 초기화
  mcpwm_gpio_init(PWM_UNIT, MCPWM0A, PIN_PWMA);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = PWM_FREQ * 2;
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  
  mcpwm_init(PWM_UNIT, PWM_TIMER, &pwm_config);
}

void setTargetSpeed(float speed, ControlMode requiredMode) {
  if (controlMode == requiredMode) {
    targetSpeed = speed;
    Serial.printf("[Speed] Set to %.1f (Mode: %s)\n", speed, 
                  requiredMode == HTTP_CONTROL ? "HTTP" : "MQTT");
  } else {
    Serial.printf("[Speed] IGNORED - Current mode: %s, Request from: %s\n",
                  controlMode == HTTP_CONTROL ? "HTTP" : "MQTT",
                  requiredMode == HTTP_CONTROL ? "HTTP" : "MQTT");
  }
}

unsigned long lastMotorUpdate = 0;

void updateMotorSpeed(bool direction)
{
  if (millis() - lastMotorUpdate < 20) { // 20ms마다 한 번씩만 속도 변경 (초당 50회)
    return;
  }
  lastMotorUpdate = millis();

  //Serial.printf("Target: %.1f, Current: %.1f\n", targetSpeed, currentSpeed);
  
  if (abs(currentSpeed - targetSpeed) > 0.1) {
    if (currentSpeed < targetSpeed) {
      currentSpeed += acceleration; // 2.5씩 증가
      if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
    } else {
      currentSpeed -= acceleration;
      if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
    }
    setMotor(abs(currentSpeed), direction); 
  }
}

void setMotor(float speed, bool direction) {
  if (direction) { 
    gpio_set_level((gpio_num_t)PIN_AIN1, 1); 
    gpio_set_level((gpio_num_t)PIN_AIN2, 0); 
  } else {         
    gpio_set_level((gpio_num_t)PIN_AIN1, 0); 
    gpio_set_level((gpio_num_t)PIN_AIN2, 1); 
  }
  mcpwm_set_duty(PWM_UNIT, PWM_TIMER, MCPWM_OPR_A, speed); 
  mcpwm_set_duty_type(PWM_UNIT, PWM_TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

// ==========================================
// 5. 웹 서버 핸들러 함수
// ==========================================

// 메인 페이지 HTML 제공
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<style>";
  html += "body { font-family: sans-serif; text-align: center; margin-top: 50px; background-color: #222; color: white; }";
  html += "h1 { margin-bottom: 30px; }";
  html += ".btn { padding: 20px 40px; font-size: 24px; margin: 10px; border: none; border-radius: 10px; color: white; cursor: pointer; width: 80%; max-width: 300px; touch-action: manipulation; }";
  html += ".btn-up { background-color: #4CAF50; }"; // 초록색
  html += ".btn-down { background-color: #f44336; }"; // 빨간색
  html += ".btn:active { transform: scale(0.95); opacity: 0.8; }";
  html += "#speed-display { font-size: 30px; margin: 20px; color: #ffeb3b; }";
  html += "</style></head><body>";
  
  html += "<h1>ESP32 Motor Control</h1>";
  html += "<p>Current Speed: <span id='speed-display'>" + String((int)currentSpeed) + "</span>%</p>";
  html += "<button class='btn btn-up' onclick=\"changeSpeed('up')\">Speed UP (+10)</button><br>";
  html += "<button class='btn btn-down' onclick=\"changeSpeed('down')\">Speed DOWN (-10)</button>";

  // JavaScript (비동기 통신)
  html += "<script>";
  html += "function changeSpeed(action) {";
  html += "  fetch('/' + action).then(response => response.text()).then(data => {";
  html += "    document.getElementById('speed-display').innerText = data;";
  html += "  });";
  html += "}";
  html += "</script>";
  
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// 속도 증가 요청 처리
void handleSpeedUp()
{
  if (controlMode != HTTP_CONTROL) {
    server.send(403, "text/plain", "MQTT Control Mode Active");
    Serial.println("[HTTP] Speed UP BLOCKED - MQTT mode active");
    return;
  }

  float newTarget = targetSpeed + 10.0;
  if (newTarget > 100.0) newTarget = 100.0;

  setTargetSpeed(newTarget, HTTP_CONTROL);
  Serial.println("Speed UP: " + String(targetSpeed));
  server.send(200, "text/plain", String((int)targetSpeed));
}

// 속도 감소 요청 처리
void handleSpeedDown() {
  if (controlMode != HTTP_CONTROL) {
    server.send(403, "text/plain", "MQTT Control Mode Active");
    Serial.println("[HTTP] Speed DOWN BLOCKED - MQTT mode active");
    return;
  }

  float newTarget = targetSpeed - 10.0;
  if (newTarget < 0.0) newTarget = 0.0;

  setTargetSpeed(newTarget, HTTP_CONTROL);
  Serial.println("Speed DOWN: " + String(targetSpeed));
  server.send(200, "text/plain", String((int)targetSpeed));
}

// ==========================================
// mqtt source
// ========== 구독 관리 ==========

void subscribe(String topic) {
  if (subscribedCount < 10) {
    subscribedTopics[subscribedCount++] = topic;
    Serial.printf("[Subscribe] Added: %s\n", topic.c_str());
  }
}

bool isSubscribed(String topic) {
  for (int i = 0; i < subscribedCount; i++) {
    if (subscribedTopics[i] == topic) {
      return true;
    }
  }
  return false;
}


// ========== 메시지 발행 ==========

void publish(String topic, uint8_t* payload, int payloadLen) {
  uint16_t msgId = nextMessageId++;
  if (nextMessageId == 0) nextMessageId = 1;
  
  // 메시지 직렬화
  int bufferSize = 11 + topic.length() + payloadLen;
  if(bufferSize > sizeof(txBuffer))
  {
    Serial.println("[Error] Tx Buffer overflow");
    return;
  }

  uint8_t* buffer = txBuffer;
  
  if (buffer == NULL) {
    Serial.println("[Error] Memory allocation failed");
    return;
  }

  int pos = 0;
  
  buffer[pos++] = START_BYTE;
  buffer[pos++] = PUBLISH;
  buffer[pos++] = (msgId >> 8) & 0xFF;
  buffer[pos++] = msgId & 0xFF;
  buffer[pos++] = AT_LEAST_ONCE;
  buffer[pos++] = (uint8_t)topic.length();
  
  for (int i = 0; i < topic.length(); i++) {
    buffer[pos++] = (uint8_t)topic[i];
  }
  
  // Payload Len (2bytes)
  buffer[pos++] = (payloadLen >> 8) & 0xFF;
  buffer[pos++] = payloadLen & 0xFF;
  
  for (int i = 0; i < payloadLen; i++) {
    buffer[pos++] = payload[i];
  }
  
  uint16_t crc = calculateCRC16(buffer + 1, pos - 1);
  buffer[pos++] = (crc >> 8) & 0xFF;
  buffer[pos++] = crc & 0xFF;
  
  buffer[pos++] = END_BYTE;
  
  // Serial2로 전송
  //Serial2.write(buffer, pos);
}

void publish(String topic, String payload) {
  publish(topic, (uint8_t*)payload.c_str(), payload.length());
}

// ========== 메시지 수신 처리 ==========


void processMessages() {
  // 수신 완료 여부만 확인 (수신 중이면 그냥 리턴)
  if(!DW1000Ng::isReceiveDone()) {
    return;
  }
  //Serial.println("[UWB] Packet Received");
  int dataLen = DW1000Ng::getReceivedDataLength();
  numReceived++;

  if (dataLen > sizeof(recvBuffer) || dataLen < 5) {
    Serial.println("[Error] Invalid packet size");
    DW1000Ng::clearReceiveStatus();
    DW1000Ng::startReceive(); // 에러 후 재시작
    return;
  }
  
  byte tempBuffer[dataLen];
  DW1000Ng::getReceivedData(tempBuffer, dataLen);

  // 수신된 데이터를 message 변수에 저장
  message = "";
  for(int i = 0; i < dataLen; i++) {
    message += (char)tempBuffer[i];
  }
  
  bool handledAsRanging = handleRangingFrame(tempBuffer, dataLen);

  if (!handledAsRanging && tempBuffer[0] == START_BYTE && tempBuffer[dataLen-1] == END_BYTE) {
    //Serial.println("[UWB] Valid Frame Received. Parsing...");
    if (parseMessage(tempBuffer, dataLen)) {
      Serial.println("[UWB] Message Parsed Successfully.");
    } else {
      Serial.println("[UWB] Parsing Failed (CRC Error or Logic).");
    }
  } else if (!handledAsRanging) {
    //Serial.println("[Warning] Frame format mismatch (Not our protocol)");
  }
  
  // 수신 처리 완료 후 다시 수신 모드로
  DW1000Ng::clearReceiveStatus();
  DW1000Ng::startReceive();
}

bool handleRangingFrame(uint8_t* data, int len) {
  if (len != UWB_RANGE_FRAME_LEN) {
    return false;
  }

  byte msgId = data[0];
  if (msgId != UWB_POLL && msgId != UWB_RANGE) {
    return false;
  }

  if (msgId == UWB_POLL) {
    rangingProtocolFailed = false;
    timePollReceived = DW1000Ng::getReceiveTimestamp();
    sendPollAckFrame();
    return true;
  }

  // UWB_RANGE
  timeRangeReceived = DW1000Ng::getReceiveTimestamp();
  if (!rangingProtocolFailed) {
    timePollSent = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
    timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
    timeRangeSent = DW1000NgUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);

    double distance = DW1000NgRanging::computeRangeAsymmetric(
        timePollSent,
        timePollReceived,
        timePollAckSent,
        timePollAckReceived,
        timeRangeSent,
        timeRangeReceived);
    distance = DW1000NgRanging::correctRange(distance);

    if (distance <= 0.0 || distance > 100.0) {
      sendRangeFailedFrame();
      return true;
    }

    sendRangeReportFrame((float)distance);
    lastComputedRangeMeters = (float)distance;

    String payload = String((float)distance, 3);
    publish("telemetry/range/last", payload);
    Serial.printf("[UWB-RANGE] %.3f m\n", (float)distance);
  } else {
    sendRangeFailedFrame();
  }

  return true;
}

void sendPollAckFrame() {
  DW1000Ng::forceTRxOff();

  memset(rangeFrame, 0, sizeof(rangeFrame));
  rangeFrame[0] = UWB_POLL_ACK;
  DW1000Ng::setTransmitData(rangeFrame, UWB_RANGE_FRAME_LEN);
  DW1000Ng::startTransmit();

  uint32_t deadline = millis() + 50; 
  while (!DW1000Ng::isTransmitDone()) {
    if (millis() > deadline) {
      Serial.println("[UWB] POLL_ACK 송신 타임아웃!");
      break; 
    }
    delay(1);
  }
  DW1000Ng::clearTransmitStatus();
  timePollAckSent = DW1000Ng::getTransmitTimestamp();
}

void sendRangeReportFrame(float meters) {
  DW1000Ng::forceTRxOff();

  memset(rangeFrame, 0, sizeof(rangeFrame));
  rangeFrame[0] = UWB_RANGE_REPORT;
  float encodedDistance = meters * DISTANCE_OF_RADIO_INV;
  memcpy(rangeFrame + 1, &encodedDistance, sizeof(float));
  rangeFrame[5] = CAR_ID;  // 차량 식별자 삽입

  DW1000Ng::setTransmitData(rangeFrame, UWB_RANGE_FRAME_LEN);
  DW1000Ng::startTransmit();

  uint32_t deadline = millis() + 50;
  while (!DW1000Ng::isTransmitDone()) {
    if (millis() > deadline) break;
    delay(1);
  }
  DW1000Ng::clearTransmitStatus();
}

void sendRangeFailedFrame() {
  DW1000Ng::forceTRxOff();

  memset(rangeFrame, 0, sizeof(rangeFrame));
  rangeFrame[0] = UWB_RANGE_FAILED;
  DW1000Ng::setTransmitData(rangeFrame, UWB_RANGE_FRAME_LEN);
  DW1000Ng::startTransmit();

  uint32_t deadline = millis() + 50;
  while (!DW1000Ng::isTransmitDone()) {
    if (millis() > deadline) break;
    delay(1);
  }
  DW1000Ng::clearTransmitStatus();
}

// UWB로 신호등에 FSM 모드 전환 알림 전송
// loop()에서 pendingModeNotify 플래그를 확인 후 호출 (processMessages 완료 후)
void sendModeNotifyFrame(uint8_t mode) {
  byte notifyFrame[UWB_RANGE_FRAME_LEN] = {0};
  notifyFrame[0] = (byte)UWB_MODE_NOTIFY;
  notifyFrame[1] = CAR_ID;
  notifyFrame[2] = mode;
  DW1000Ng::setTransmitData(notifyFrame, UWB_RANGE_FRAME_LEN);
  DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);
  // millis 기반 타임아웃 (delay() 사용 금지 — CLAUDE.md §4)
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

bool parseMessage(uint8_t* data, int len) {
  int pos = 1; 
  
  uint8_t msgType = data[pos++];
  
  uint16_t msgId = (data[pos] << 8) | data[pos + 1];
  pos += 2;
  
  uint8_t qos = data[pos++];
  uint8_t topicLen = data[pos++];
  
  if (pos + topicLen + 2 > len) return false;
  
  String topic = "";
  for (int i = 0; i < topicLen; i++) {
    topic += (char)data[pos++];
  }
  
  uint16_t payloadLen = (data[pos] << 8) | data[pos + 1];
  pos += 2;
  
  if (pos + payloadLen + 3 > len) return false;
  
  uint8_t* payload = data + pos;
  pos += payloadLen;
  
  uint16_t receivedCRC = (data[pos] << 8) | data[pos + 1];
  uint16_t calculatedCRC = calculateCRC16(data + 1, pos - 1);
  
  if (receivedCRC != calculatedCRC) {
    Serial.println("[Error] CRC mismatch");
    return false; 
  }
  
  if (msgType == PUBLISH) {
    handlePublish(msgId, qos, topic, payload, payloadLen);
  } else if (msgType == ACK) {
    handleAck(msgId);
  }
  
  return true;
}

void handlePublish(uint16_t msgId, uint8_t qos, String topic, uint8_t* payload, int len) {
  if (!isSubscribed(topic)) return;
  
  if (qos == AT_LEAST_ONCE) {
    sendAck(msgId);
  }
  
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
  }
  else if (topic == "command/led") {
    if (len > 0) {
      if (payload[0] == '1' || payload[0] == 1) {
        digitalWrite(2, HIGH); // ESP32 Builtin LED
        Serial.println("[CMD] LED ON");
      } else {
        digitalWrite(2, LOW);
        Serial.println("[CMD] LED OFF");
      }
    }
  }
  else if (topic == "command/motor") {
    if (len > 0) {
      // payload를 null-terminated string으로 변환
      char buffer[32];
      int copyLen = (len < 31) ? len : 31;
      memcpy(buffer, payload, copyLen);
      buffer[copyLen] = '\0';
      
      float speed = atof(buffer); // 문자열을 float으로 변환
      
      // 속도 범위 검증 (0.0 ~ 100.0)
      if (speed < 0.0) speed = 0.0;
      if (speed > 100.0) speed = 100.0;
      
      setTargetSpeed(speed, MQTT_CONTROL);
    }
  }
  else if (topic == "command/range_probe") {
    if (lastComputedRangeMeters >= 0.0f) {
      publish("telemetry/range/last", String(lastComputedRangeMeters, 3));
    } else {
      publish("telemetry/range/last", "-1");
    }
  }
  else {
    // 디버깅용: 수신된 페이로드 출력
    Serial.printf("[Recv] Topic: %s, Payload Len: %d\n", topic.c_str(), len);
  }
}

void handleAck(uint16_t msgId) {
  Serial.printf("[ACK] Message ID: %d\n", msgId);
}

void sendAck(uint16_t msgId) {
  uint8_t buffer[11];
  int pos = 0;
  
  buffer[pos++] = START_BYTE;
  buffer[pos++] = ACK;
  buffer[pos++] = (msgId >> 8) & 0xFF;
  buffer[pos++] = msgId & 0xFF;
  buffer[pos++] = AT_MOST_ONCE;
  buffer[pos++] = 0; // topic len
  buffer[pos++] = 0; // payload len high
  buffer[pos++] = 0; // payload len low
  
  uint16_t crc = calculateCRC16(buffer + 1, pos - 1);
  buffer[pos++] = (crc >> 8) & 0xFF;
  buffer[pos++] = crc & 0xFF;
  buffer[pos++] = END_BYTE;
  
  DW1000Ng::setTransmitData(buffer, pos);
  DW1000Ng::startTransmit();
  
  // 전송이 끝날 때까지 기다렸다가 다시 수신 모드로 돌아가야 함
  while(!DW1000Ng::isTransmitDone()) {
      delay(1);
  }
  DW1000Ng::clearTransmitStatus();
  DW1000Ng::startReceive(); // 전송 후 다시 수신 모드 복귀
}

uint16_t calculateCRC16(uint8_t* data, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc = crc << 1;
    }
  }
  return crc;
}

void setup() {
  Serial.begin(9600);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);
  DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);

  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  
  DW1000Ng::setDeviceAddress(CAR_ID); // 수신기 주소
  DW1000Ng::setNetworkId(10);    // 네트워크 ID

  DW1000Ng::setAntennaDelay(16436);
  Serial.println(F("Committed configuration ..."));

  // // DWM 확인
  char msg[128];
  DW1000Ng::getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000Ng::getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);

  ////////////////////////////////////
  // 모터 초기화
  initMotor();
  setMotor(0, true); // 초기 정지 상태


  ///////////wifi 연결
  Serial.println("Setting up Access Point...");
  
  // SSID와 Password 설정 (비밀번호는 8자리 이상이어야 함)
  // password를 NULL로 하면 비밀번호 없는 개방형 와이파이가 됩니다.
  WiFi.softAP(ssid, password); 

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP); // 보통 192.168.4.1 로 잡힙니다.
  
  server.on("/", handleRoot);
  server.on("/up", handleSpeedUp);
  server.on("/down", handleSpeedDown);

  server.begin();
  Serial.println("Web server started");

  // MQTT 구독
  subscribe("command/controlled");
  subscribe("command/motor");
  subscribe("command/led");
  subscribe("command/range_probe");
  subscribe("status/ranging");

  // DWM 수신 시작 (최초 한 번만)
  DW1000Ng::startReceive();
  Serial.println("DW1000 Receive mode started");
}

void loop() {
  server.handleClient(); // 클라이언트 요청 처리

  processMessages();

  // processMessages() 완료 후 UWB MODE_NOTIFY 전송 (수신 버퍼 오염 방지)
  if (pendingModeNotify) {
    pendingModeNotify = false;
    sendModeNotifyFrame(pendingMode);
  }

  // 모터 속도 업데이트
  updateMotorSpeed(currentDirection);
}
