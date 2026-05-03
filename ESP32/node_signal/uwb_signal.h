#pragma once
#include <Arduino.h>

// ============================================================
//  uwb_signal.h  —  신호등 노드 전용 UWB Proto-B 송신
//
//  기존 팀 Proto-B 프레임 포맷 (node_car.ino 호환):
//  [ 0x7E | PUBLISH(0x01) | msgId_HI | msgId_LO | QoS(0x00)
//    | topicLen | topic... | payloadLen_HI | payloadLen_LO
//    | payload... | CRC16_HI | CRC16_LO | 0x7F ]
//
//  새 토픽:
//    telemetry/signal_state  payload: [stateByte, remainSec]
//    telemetry/signal_end    payload: [0x00]
// ============================================================

// ── Proto-B 상수 ─────────────────────────────
#define UWB_START_BYTE  0x7E
#define UWB_END_BYTE    0x7F
#define UWB_PUBLISH     0x01
#define UWB_QOS_0       0x00   // AT_MOST_ONCE: 차량 ACK 없음

// ── 토픽 ─────────────────────────────────────
#define TOPIC_SIGNAL_STATE  "telemetry/signal_state"
#define TOPIC_SIGNAL_END    "telemetry/signal_end"

// ── signal_state payload 비트필드 ─────────────
// bit0-1 : car  (0=GREEN, 1=YELLOW, 2=RED)
// bit2-3 : ped  (0=WALK,  1=RED)
// bit4   : pedExtend
// bit5   : emergency
#define CAR_LIGHT_GREEN   0
#define CAR_LIGHT_YELLOW  1
#define CAR_LIGHT_RED     2
#define PED_LIGHT_WALK    0
#define PED_LIGHT_RED     1

#define ENCODE_SIGNAL_STATE(car, ped, pedExt, emerg) \
    ( ((car)&0x03) | (((ped)&0x03)<<2) | ((pedExt?1:0)<<4) | ((emerg?1:0)<<5) )

#define DECODE_CAR(b)        ((b)&0x03)
#define DECODE_PED(b)        (((b)>>2)&0x03)
#define DECODE_PED_EXTEND(b) (((b)>>4)&0x01)
#define DECODE_EMERGENCY(b)  (((b)>>5)&0x01)

// ── 초기화 (setup에서 1회) ────────────────────
// nodeId: 1~3
void uwb_signal_init(uint8_t nodeId);

// ── signal_state 송신 ─────────────────────────
// stateByte: ENCODE_SIGNAL_STATE(...)
// remainSec: 현재 phase 남은 시간(초)
bool uwb_signal_send_state(uint8_t stateByte, uint8_t remainSec);

// ── signal_end 송신 (세션 종료) ───────────────
bool uwb_signal_send_end();

// ── 수신 폴링 (loop에서 호출) ─────────────────
// 새 프레임 있으면 true, topicOut/payloadOut에 결과
// 차량이 EMERGENCY 요청 같은 걸 보냈을 때 처리용
bool uwb_signal_poll_rx(char* topicOut,    uint8_t topicMaxLen,
                        uint8_t* payloadOut, uint8_t* payloadLenOut);
