// node_template.ino — ESP32 UART 노드 메인 스케치
//
// ── 업로드 전 반드시 확인할 사항 ────────────────────────────
//   uart_comm.h 상단의 아래 값을 노드에 맞게 수정:
//     #define NODE_ID      1   ← 노드 번호 (1 / 2 / 3)
//     #define UART_TX_PIN  25  ← RPi RX 에 연결된 GPIO
//     #define UART_RX_PIN  26  ← RPi TX 에 연결된 GPIO
//     #define UART_BAUD    115200
//
// ── Arduino IDE 보드 설정 ────────────────────────────────────
//   도구 → 보드 → esp32 by Espressif → ESP32 Dev Module
//   도구 → 포트 → ESP32 가 연결된 COM 포트
//
// ── UART 구성 ────────────────────────────────────────────────
//   Serial  (UART0): USB 디버그 전용 (시리얼 모니터)
//   Serial1 (UART1): 라즈베리파이 통신용 (HardwareSerial)

#include "uart_comm.h"

UartComm uartComm;

void setup() {
    // UART0 — 디버그 출력 (시리얼 모니터 115200 설정)
    Serial.begin(115200);
    delay(500);  // 시리얼 모니터 연결 대기 (setup 1회만 허용)

    Serial.printf("=== ESP32 Node%d 부팅 ===\n", NODE_ID);
    Serial.printf("    UART1 TX=GPIO%d  RX=GPIO%d  %d baud\n",
                  UART_TX_PIN, UART_RX_PIN, UART_BAUD);

    // UART1 — 라즈베리파이 통신 초기화
    uartComm.begin();

    Serial.printf("=== 초기화 완료. PING/RANGE 요청 대기 중 ===\n");
}

void loop() {
    // CLAUDE.md §4: loop() 내 수신 처리 패턴
    //   - delay() 호출 없음
    //   - available() → read() → FSM 파서 투입
    uartComm.process();
}
