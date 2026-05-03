#include "uwb_signal.h"
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <SPI.h>
#include <string.h>

// ── 핀 (기존 팀 고정값) ──────────────────────
#define UWB_PIN_SCK   18
#define UWB_PIN_MISO  19
#define UWB_PIN_MOSI  23
#define UWB_PIN_SS     5
#define UWB_PIN_RST   22
#define UWB_NETWORK_ID    10
#define UWB_ANTENNA_DELAY 16436

// ── 기존 팀과 동일한 DEFAULT_CONFIG ──────────
static device_configuration_t UWB_CONFIG = {
    false, true, true, true, false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

static uint8_t  s_nodeId   = 0;
static uint16_t s_msgId    = 1;
static uint8_t  s_txBuf[128];
static uint8_t  s_rxBuf[256];

// ── CRC-16 CCITT (node_car.ino calculateCRC16와 동일) ────────
static uint16_t crc16(const uint8_t* data, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc <<= 1;
        }
    }
    return crc;
}

// ── Proto-B 프레임 빌드 (node_car.ino publish() 완전 동일) ───
// [ 0x7E | PUBLISH | msgId_HI | msgId_LO | QoS
//   | topicLen | topic... | payloadLen_HI | payloadLen_LO
//   | payload... | CRC16_HI | CRC16_LO | 0x7F ]
static size_t build_publish(uint8_t* buf, size_t bufSize,
                             const char* topic,
                             const uint8_t* payload, int payloadLen) {
    uint8_t topicLen = (uint8_t)strlen(topic);
    // 11 = SOF+type+msgId(2)+QoS+topicLen+payloadLen(2)+CRC(2)+EOF
    size_t needed = 11 + topicLen + payloadLen;
    if (bufSize < needed) return 0;

    uint16_t msgId = s_msgId++;
    if (s_msgId == 0) s_msgId = 1;

    int pos = 0;
    buf[pos++] = UWB_START_BYTE;
    buf[pos++] = UWB_PUBLISH;
    buf[pos++] = (msgId >> 8) & 0xFF;
    buf[pos++] =  msgId       & 0xFF;
    buf[pos++] = UWB_QOS_0;
    buf[pos++] = topicLen;
    memcpy(buf + pos, topic, topicLen); pos += topicLen;
    buf[pos++] = (payloadLen >> 8) & 0xFF;
    buf[pos++] =  payloadLen       & 0xFF;
    if (payloadLen > 0) {
        memcpy(buf + pos, payload, payloadLen); pos += payloadLen;
    }
    // CRC: buf[1] ~ buf[pos-1] (START_BYTE 제외)
    uint16_t crc = crc16(buf + 1, pos - 1);
    buf[pos++] = (crc >> 8) & 0xFF;
    buf[pos++] =  crc       & 0xFF;
    buf[pos++] = UWB_END_BYTE;

    return pos;
}

// ── TX 헬퍼 (README 버그#2: forceTRxOff 필수) ────────────────
static bool uwb_tx(uint8_t* buf, size_t len) {
    DW1000Ng::forceTRxOff();                        // 버그#2: RX→TX 전환
    DW1000Ng::setTransmitData(buf, len);
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    uint32_t t0 = millis();
    while (!DW1000Ng::isTransmitDone()) {
        if (millis() - t0 > 100) {
            Serial.println("[UWB] TX timeout");
            DW1000Ng::forceTRxOff();
            DW1000Ng::startReceive();
            return false;
        }
    }
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::startReceive();                       // TX 후 즉시 RX 복귀
    return true;
}

// ── 초기화 ───────────────────────────────────
void uwb_signal_init(uint8_t nodeId) {
    s_nodeId = nodeId;
    SPI.begin(UWB_PIN_SCK, UWB_PIN_MISO, UWB_PIN_MOSI);
    DW1000Ng::initializeNoInterrupt(UWB_PIN_SS, UWB_PIN_RST);
    DW1000Ng::applyConfiguration(UWB_CONFIG);
    DW1000Ng::setNetworkId(UWB_NETWORK_ID);
    DW1000Ng::setDeviceAddress(nodeId);
    DW1000Ng::setAntennaDelay(UWB_ANTENNA_DELAY);
    DW1000Ng::startReceive();
    Serial.printf("[UWB] init OK. nodeId=%d\n", nodeId);
}

// ── signal_state 송신 ────────────────────────
bool uwb_signal_send_state(uint8_t stateByte, uint8_t remainSec) {
    uint8_t payload[2] = { stateByte, remainSec };
    size_t frameLen = build_publish(s_txBuf, sizeof(s_txBuf),
                                    TOPIC_SIGNAL_STATE, payload, 2);
    if (frameLen == 0) { Serial.println("[UWB] build failed"); return false; }

    bool ok = uwb_tx(s_txBuf, frameLen);
    Serial.printf("[UWB] signal_state 0x%02X rem=%ds %s\n",
                  stateByte, remainSec, ok ? "OK" : "FAIL");
    return ok;
}

// ── signal_end 송신 ──────────────────────────
bool uwb_signal_send_end() {
    uint8_t payload[1] = { 0x00 };
    size_t frameLen = build_publish(s_txBuf, sizeof(s_txBuf),
                                    TOPIC_SIGNAL_END, payload, 1);
    if (frameLen == 0) return false;

    bool ok = uwb_tx(s_txBuf, frameLen);
    Serial.printf("[UWB] signal_end %s\n", ok ? "OK" : "FAIL");
    return ok;
}

// ── 수신 폴링 ────────────────────────────────
// README 버그#3: clearReceiveStatus()를 TX 호출 전에
bool uwb_signal_poll_rx(char* topicOut,    uint8_t topicMaxLen,
                        uint8_t* payloadOut, uint8_t* payloadLenOut) {
    if (!DW1000Ng::isReceiveDone()) return false;

    int dataLen = DW1000Ng::getReceivedDataLength();
    if (dataLen <= 0 || dataLen > (int)sizeof(s_rxBuf)) {
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive();
        return false;
    }

    DW1000Ng::getReceivedData(s_rxBuf, dataLen);
    DW1000Ng::clearReceiveStatus();   // 버그#3: TX 전에 clear
    DW1000Ng::startReceive();

    // node_car.ino parseMessage와 동일한 포맷으로 파싱
    if (dataLen < 11) return false;
    if (s_rxBuf[0] != UWB_START_BYTE) return false;
    if (s_rxBuf[dataLen - 1] != UWB_END_BYTE) return false;

    // msgType(1), msgId(2), QoS(1), topicLen(1)
    uint8_t topicLen = s_rxBuf[5];
    if (dataLen < 11 + topicLen) return false;

    uint16_t payloadLen = ((uint16_t)s_rxBuf[6 + topicLen] << 8)
                        |  (uint16_t)s_rxBuf[7 + topicLen];
    int expectedLen = 8 + topicLen + payloadLen + 3;  // +CRC(2)+EOF(1)
    if (expectedLen != dataLen) return false;

    // CRC 검증
    uint16_t crcRecv = ((uint16_t)s_rxBuf[dataLen - 3] << 8)
                     |  (uint16_t)s_rxBuf[dataLen - 2];
    uint16_t crcCalc = crc16(s_rxBuf + 1, dataLen - 4);
    if (crcRecv != crcCalc) {
        Serial.println("[UWB] CRC mismatch");
        return false;
    }

    // topic 복사
    uint8_t copyLen = (topicLen < topicMaxLen - 1) ? topicLen : topicMaxLen - 1;
    memcpy(topicOut, s_rxBuf + 6, copyLen);
    topicOut[copyLen] = '\0';

    // payload 복사
    *payloadLenOut = (uint8_t)payloadLen;
    if (payloadLen > 0) memcpy(payloadOut, s_rxBuf + 8 + topicLen, payloadLen);

    return true;
}
