#include "uart_comm.h"
#include <DW1000NgConstants.hpp>
#include <SPI.h>
#include <cstring>

// ── 초기화 ────────────────────────────────────────────────────
void UartComm::begin() {
    proto_parser_reset(&_parser);
    // HardwareSerial(1) = UART1
    // begin(baud, config, rxPin, txPin)
    _serial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    SPI.begin(UWB_PIN_SCK, UWB_PIN_MISO, UWB_PIN_MOSI, UWB_PIN_SS);
    DW1000Ng::initializeNoInterrupt(UWB_PIN_SS, UWB_PIN_RST);

    device_configuration_t uwbConfig = {
        false,
        true,
        true,
        true,
        false,
        SFDMode::STANDARD_SFD,
        Channel::CHANNEL_5,
        DataRate::RATE_6800KBPS,
        PulseFrequency::FREQ_16MHZ,
        PreambleLength::LEN_128,
        PreambleCode::CODE_3
    };

    DW1000Ng::applyConfiguration(uwbConfig);
    DW1000Ng::setNetworkId(10);
    DW1000Ng::setDeviceAddress((uint16_t)NODE_ID);
    DW1000Ng::setAntennaDelay(16436);
    DW1000Ng::startReceive();

    Serial.printf("[Node%d] UWB init complete (SS=%d RST=%d IRQ=%d)\n",
                  NODE_ID, UWB_PIN_SS, UWB_PIN_RST, UWB_PIN_IRQ);
}

// ── 수신 처리 (loop() 에서 호출) ─────────────────────────────
// CLAUDE.md §4: loop() 수신 패턴
//   while (available()) → read() → FSM feed()
//   delay() 호출 없이 즉시 반환
void UartComm::process() {
    while (_serial.available()) {
        uint8_t b = (uint8_t)_serial.read();

        uint8_t payload[PROTO_MAX_PAYLOAD];
        uint8_t plen = 0;

        if (proto_parser_feed(&_parser, b, payload, &plen)) {
            handle_packet(payload, plen);
        }
    }

    process_uwb();
}

// ── 수신 패킷 처리 ───────────────────────────────────────────
void UartComm::handle_packet(const uint8_t *payload, uint8_t len) {
    if (len < 1) return;

    if (payload[0] == CMD_PING && len >= 3) {
        // 빅 엔디안 역직렬화 (CLAUDE.md §5: 비트 시프트 연산 필수)
        const uint16_t seq = ((uint16_t)payload[1] << 8)
                           |  (uint16_t)payload[2];

        Serial.printf("[Node%d] PING recv: SEQ=%u\n", NODE_ID, seq);

        // PONG 응답: [CMD_PONG][SEQ_HI][SEQ_LO][NODE_ID]
        uint8_t pong[4];
        pong[0] = CMD_PONG;
        pong[1] = (seq >> 8) & 0xFF;   // 빅 엔디안 직렬화 (CLAUDE.md §5)
        pong[2] =  seq       & 0xFF;
        pong[3] = (uint8_t)NODE_ID;

        send(pong, sizeof(pong));
        Serial.printf("[Node%d] PONG sent: SEQ=%u\n", NODE_ID, seq);

    } else if (payload[0] == CMD_RANGE_REQUEST && len >= 3) {
        const uint16_t seq = ((uint16_t)payload[1] << 8)
                           |  (uint16_t)payload[2];
        start_ranging(seq);

    } else {
        Serial.printf("[Node%d] 알 수 없는 CMD=0x%02X len=%u\n",
                      NODE_ID, payload[0], len);
    }
}

void UartComm::start_ranging(uint16_t seq) {
    if (_rangeState != RangeState::IDLE) {
        Serial.printf("[Node%d] range busy, request ignored (SEQ=%u)\n", NODE_ID, seq);
        return;
    }

    _activeSeq = seq;
    _rangeState = RangeState::WAIT_POLL_ACK;
    _rangeDeadlineMs = millis() + 300;
    send_poll();
    Serial.printf("[Node%d] range start (SEQ=%u)\n", NODE_ID, _activeSeq);
}

void UartComm::send_poll() {
    memset(_uwbData, 0, sizeof(_uwbData));
    _uwbData[0] = UWB_POLL;
    DW1000Ng::setTransmitData(_uwbData, UWB_FRAME_LEN);
    DW1000Ng::startTransmit();
}

void UartComm::send_range() {
    memset(_uwbData, 0, sizeof(_uwbData));
    _uwbData[0] = UWB_RANGE;

    byte futureTimeBytes[LENGTH_TIMESTAMP];
    uint64_t timeRangeSent = DW1000Ng::getSystemTimestamp();
    timeRangeSent += DW1000NgTime::microsecondsToUWBTime(3000);
    DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
    DW1000Ng::setDelayedTRX(futureTimeBytes);
    timeRangeSent += DW1000Ng::getTxAntennaDelay();

    DW1000NgUtils::writeValueToBytes(_uwbData + 1, _timePollSent, LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(_uwbData + 6, _timePollAckReceived, LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(_uwbData + 11, timeRangeSent, LENGTH_TIMESTAMP);

    DW1000Ng::setTransmitData(_uwbData, UWB_FRAME_LEN);
    DW1000Ng::startTransmit(TransmitMode::DELAYED);
}

void UartComm::process_uwb() {
    if (_rangeState == RangeState::IDLE) {
        return;
    }

    if ((int32_t)(millis() - _rangeDeadlineMs) > 0) {
        fail_ranging("timeout");
        return;
    }

    if (DW1000Ng::isTransmitDone()) {
        DW1000Ng::clearTransmitStatus();
        DW1000Ng::startReceive();
    }

    if (!DW1000Ng::isReceiveDone()) {
        return;
    }

    int dataLen = DW1000Ng::getReceivedDataLength();
    if (dataLen != UWB_FRAME_LEN) {
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive();
        fail_ranging("invalid frame len");
        return;
    }

    byte frame[UWB_FRAME_LEN];
    DW1000Ng::getReceivedData(frame, UWB_FRAME_LEN);
    const byte msgId = frame[0];

    if (_rangeState == RangeState::WAIT_POLL_ACK) {
        if (msgId != UWB_POLL_ACK) {
            DW1000Ng::clearReceiveStatus();
            DW1000Ng::startReceive();
            fail_ranging("expected POLL_ACK");
            return;
        }

        _timePollSent = DW1000Ng::getTransmitTimestamp();
        _timePollAckReceived = DW1000Ng::getReceiveTimestamp();
        _rangeState = RangeState::WAIT_RANGE_REPORT;
        _rangeDeadlineMs = millis() + 300;
        DW1000Ng::clearReceiveStatus();
        send_range();
        return;
    }

    if (_rangeState == RangeState::WAIT_RANGE_REPORT) {
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive();

        if (msgId == UWB_RANGE_FAILED) {
            fail_ranging("anchor reported failure");
            return;
        }
        if (msgId != UWB_RANGE_REPORT) {
            fail_ranging("expected RANGE_REPORT");
            return;
        }

        float rawDistance = 0.0f;
        memcpy(&rawDistance, frame + 1, sizeof(float));
        const float distanceMeters = rawDistance / DISTANCE_OF_RADIO_INV;
        const uint16_t distanceCm = meters_to_cm(distanceMeters);
        const int16_t rssiCentiDbm = (int16_t)(DW1000Ng::getReceivePower() * 100.0f);

        report_range(distanceCm, rssiCentiDbm);
        _rangeState = RangeState::IDLE;
        Serial.printf("[Node%d] range done: %u cm, RSSI=%d cdbm\n",
                      NODE_ID, distanceCm, (int)rssiCentiDbm);
    }
}

void UartComm::report_range(uint16_t distance_cm, int16_t rssi_centi_dbm) {
    uint8_t payload[6];
    payload[0] = CMD_RANGE_REPORT;
    payload[1] = (uint8_t)NODE_ID;
    payload[2] = (distance_cm >> 8) & 0xFF;
    payload[3] = distance_cm & 0xFF;
    payload[4] = (rssi_centi_dbm >> 8) & 0xFF;
    payload[5] = rssi_centi_dbm & 0xFF;
    send(payload, sizeof(payload));
}

void UartComm::fail_ranging(const char *reason) {
    report_range(0xFFFF, (int16_t)0x8000);
    _rangeState = RangeState::IDLE;
    DW1000Ng::forceTRxOff();
    DW1000Ng::startReceive();
    Serial.printf("[Node%d] range failed: %s\n", NODE_ID, reason);
}

uint16_t UartComm::meters_to_cm(float meters) {
    if (meters <= 0.0f) {
        return 0;
    }
    float cm = meters * 100.0f;
    if (cm > 65534.0f) {
        cm = 65534.0f;
    }
    return (uint16_t)(cm + 0.5f);
}

// ── 송신 ─────────────────────────────────────────────────────
void UartComm::send(const uint8_t *payload, uint8_t len) {
    uint8_t frame[PROTO_MAX_FRAME];
    const size_t frame_len = proto_build_frame(payload, len,
                                               frame, sizeof(frame));
    if (frame_len == 0) {
        Serial.println("[uart] 프레임 빌드 실패");
        return;
    }
    _serial.write(frame, frame_len);
}
