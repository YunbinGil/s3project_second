# UWB 삼변측량 기반 차량 위치 추적 시스템

> Raspberry Pi (중앙 모니터) ↔ 신호등 ESP32 ×3 (UART) ↔ 차량 ESP32 ×1 (UWB)

DWM1000 UWB 모듈을 활용한 실시간 차량 위치 추적 + 신호등 자동 제어 시스템입니다.

---

## 시스템 구성

```
              차량 ESP32 (독립, RPi 미연결)
              [DWM1000 + Motor + WiFi AP]
                 /       |        \
           UWB  /   UWB  |   UWB   \
              /          |           \
  신호등 ESP32-1   신호등 ESP32-2   신호등 ESP32-3
  [DWM1000]        [DWM1000]        [DWM1000]
       |                |                |
   UART(PL011)     UART(PL011)     UART(PL011)
       |                |                |
       └────────  Raspberry Pi  ─────────┘
               (중앙 위치 계산기 / 모니터)
```

| 구성 요소 | 사양 |
|-----------|------|
| **호스트** | Raspberry Pi 4/5, Linux, C++17 |
| **신호등 노드 ×3** | ESP32 + DWM1000 UWB + UART (Arduino) |
| **차량 노드 ×1** | ESP32 + DWM1000 UWB + TB6612FNG 모터 + WiFi AP (Arduino) |

### 데이터 흐름

1. **위치 추적**: 차량 ↔ 신호등 ×3 (UWB 거리 측정) → 신호등 → UART → RPi (삼변측량)
2. **자동 제어**: RPi → UART → 신호등 → UWB → 차량 (모터 속도 변경)
3. **수동 제어**: 스마트폰 → WiFi AP → 차량 WebServer (HTTP)

---

## 디렉토리 구조

```
s3project/
├── CLAUDE.md                  # AI 코딩 에이전트 프로젝트 헌장
├── README.md                  # (이 파일)
├── preCode/                   # 이전 프로젝트 참조 코드 (수정 금지)
│
├── raspberryPI/               # RPi 중앙 서버 (Remote SSH 빌드)
│   ├── CMakeLists.txt
│   ├── scripts/
│   │   └── setup_uart.sh      # UART overlay + udev 설치
│   ├── udev/
│   │   └── 99-esp32-uart.rules
│   └── src/
│       ├── main.cpp           # epoll 루프 + 삼변측량 계산
│       ├── uart_manager.h/cpp # epoll 이벤트 루프
│       └── protocol.h/cpp     # CRC-8 룩업 테이블 / FSM 파서
│
└── ESP32/                     # ESP32 노드 (Arduino IDE 빌드)
    ├── node_template/         # 신호등 노드 ×3 공통 스케치
    │   ├── node_template.ino
    │   ├── uart_comm.h/cpp    # UART1 래퍼 (GPIO 25/26)
    │   ├── protocol.h/cpp     # CRC-8 FSM 파서
    │   ├── uwb_comm.h/cpp     # DWM1000 UWB 래퍼
    │   └── uwb_protocol.h/cpp # CRC-16 토픽 파서
    │
    ├── node_signal/           # 신호등 노드 실제 구현 (node_template 확장)
    │   ├── node_signal.ino    # FSM + LED + UWB signal_state + UART
    │   ├── uart_comm.h/cpp    # UART1 래퍼 (GPIO 25/26)
    │   ├── protocol.h/cpp     # CRC-8 FSM 파서
    │   ├── uwb_signal.h/cpp   # UWB signal_state 송수신
    │   └── (uwb_signal 관련)
    │
    └── node_car/              # 차량 노드 ×1
        ├── node_car.ino
        ├── uwb_comm.h/cpp     # DWM1000 UWB 래퍼
        ├── uwb_protocol.h/cpp # CRC-16 토픽 파서
        ├── motor_ctrl.h/cpp   # MCPWM + TB6612FNG
        ├── web_server.h/cpp   # WiFi AP + WebServer
        └── fsm_mode.h/cpp     # FSM 제어 모드
```

---

## 사전 준비

### 하드웨어

- Raspberry Pi 4 또는 5
- ESP32 Dev Module × 4
- DWM1000 UWB 모듈 × 4
- TB6612FNG 모터 드라이버 × 1
- DC 모터 × 1
- 점퍼 와이어, 브레드보드

### 소프트웨어

| 환경 | 필수 설치 |
|------|----------|
| **Raspberry Pi** | `cmake` (≥ 3.16), `g++` (C++17 지원), `git` |
| **로컬 PC** | Arduino IDE 2.x, `esp32 by Espressif` 보드 패키지, `DW1000Ng` 라이브러리 |

---

## 핀 연결

### RPi ↔ 신호등 ESP32 (UART)

> TX↔RX 교차 연결. RPi / ESP32 모두 3.3V 로직이므로 레벨 시프터 불필요.

| RPi UART | RPi TX (BCM) | RPi RX (BCM) | ESP32 UART0 RX | ESP32 UART0 TX | 대상 |
|----------|:------------:|:------------:|:--------------:|:--------------:|------|
| UART2 | GPIO 0 | GPIO 1 | GPIO 16 | GPIO 17 | 신호등 1 |
| UART3 | GPIO 4 | GPIO 5 | GPIO 16 | GPIO 17 | 신호등 2 |
| UART4 | GPIO 8 | GPIO 9 | GPIO 16 | GPIO 17 | 신호등 3 |

**+ 각 ESP32의 GND ↔ RPi GND 반드시 연결**

### 신호등/차량 ESP32 ↔ DWM1000 (SPI)

| DWM1000 | ESP32 GPIO |
|:-------:|:---------:|
| SCK | 18 |
| MISO | 19 |
| MOSI | 23 |
| SS (CS) | 5 |
| RST | 22 |
| IRQ | 4 (선택) |
| VCC | **3.3V** (5V 금지) |
| GND | GND |

### 차량 ESP32 ↔ TB6612FNG (모터)

| TB6612FNG | ESP32 GPIO | 기능 |
|:---------:|:---------:|------|
| PWMA | 15 | 속도 (MCPWM) |
| AIN1 | 27 | 방향 비트 1 |
| AIN2 | 13 | 방향 비트 2 |
| STBY | 14 | HIGH = 활성 |
| VM | 외부 5~12V | 모터 전원 |
| VCC | 3.3V | 로직 전원 |
| GND | GND (공통) | |
| AO1/AO2 | DC 모터 | 출력 |

---

## 빌드 및 실행

### Raspberry Pi (Remote SSH 터미널)

#### 1. 최초 1회: UART 환경 설정

```bash
cd raspberryPI
sudo bash scripts/setup_uart.sh
```

이 스크립트는 다음을 수행합니다:
- `/boot/firmware/config.txt`에 `dtoverlay=uart2,3,4` 추가
- udev 규칙 (`99-esp32-uart.rules`) 설치 → `/dev/ttyESP_Node1~3` 심볼릭 링크 생성

**설정 후 반드시 재부팅:**
```bash
sudo reboot
```

재부팅 후 심볼릭 링크 확인:
```bash
ls -la /dev/ttyESP_Node*
```

#### 2. 빌드

```bash
cd raspberryPI
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

정상 빌드 시 `uart_test` 실행 파일이 생성됩니다.

#### 3. 실행

```bash
# udev 심볼릭 링크 경로를 인자로 전달
./uart_test /dev/ttyESP_Node1 /dev/ttyESP_Node2 /dev/ttyESP_Node3
```

또는 기본 경로 사용 (소스에서 설정된 경우):
```bash
./uart_test
```
실행하면 epoll 이벤트 루프가 동작하며:
- 2초마다 각 신호등 노드에 `PING` 전송
- `PONG` 응답 수신 시 콘솔 출력
- `CMD_RANGE_REPORT` 수신 시 삼변측량 계산 후 차량 좌표 출력

**종료:** `Ctrl+C` (SIGINT)

---

### ESP32 — 신호등 노드 (Arduino IDE, 로컬 PC)

#### 1. Arduino IDE 설정

1. **보드 패키지 설치**:
   - `파일` → `기본 설정` → 추가 보드 매니저 URL에 다음 추가:
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - `도구` → `보드 매니저` → `esp32 by Espressif` 설치

2. **DW1000Ng 라이브러리 설치**:
   - `스케치` → `라이브러리 포함` → `라이브러리 관리` → `DW1000Ng` 검색 후 설치
   - 또는 GitHub에서 F-Army/DW1000Ng zip 다운로드 후 수동 설치

#### 2. 노드 ID 설정

`ESP32/node_template/uart_comm.h` 상단의 `NODE_ID`를 노드에 맞게 수정합니다:

```cpp
#define NODE_ID      1   // 신호등 1: 1, 신호등 2: 2, 신호등 3: 3
#define UART_TX_PIN  25  // DWM1000 SPI 핀 충돌 회피 (GPIO 16/17 사용 불가)
#define UART_RX_PIN  26
#define UART_BAUD    115200
```

> `UART_TX_PIN`과 `UART_RX_PIN`은 **25/26 고정** (변경 금지)

#### 3. 업로드

1. `파일` → `열기` → `ESP32/node_template/node_template.ino`
2. `도구` → `보드` → `ESP32` → `DOIT ESP32 DEVKIT V1`  
3. `도구` → `포트` → ESP32가 연결된 COM 포트 선택
4. `스케치` → `업로드` (`Ctrl+U`)

주의 uart 있으면 업로드가 안됨.

**3대 신호등 모두 동일한 코드를 업로드하되, `NODE_ID`만 1/2/3으로 변경합니다.**

#### 4. 시리얼 모니터 확인

- `도구` → `시리얼 모니터` (115200 baud)
- 정상 부팅 시 출력:
  ```
  === ESP32 Node1 부팅 ===
      UART1 TX=GPIO25  RX=GPIO26  115200 baud
  === 초기화 완료. PING 대기 중 ===
  ```

---

### ESP32 — 차량 노드 (Arduino IDE, 로컬 PC)

#### 1. 업로드

1. `파일` → `열기` → `ESP32/node_car/node_car.ino`
2. `도구` → `보드` → `esp32 by Espressif` → **ESP32 Dev Module**
3. `도구` → `포트` → 차량 ESP32가 연결된 COM 포트 선택
4. `스케치` → `업로드` (`Ctrl+U`)

#### 2. 동작 확인

- **WiFi AP** 자동 생성 (SSID/비밀번호는 `web_server.h`에서 설정)
- 스마트폰으로 WiFi 접속 후 브라우저에서 `http://192.168.4.1` 접속
- 웹 UI에서 속도 제어 가능 (`/up`, `/down`)

#### 3. 제어 모드

| 모드 | 전환 조건 | 입력 소스 |
|------|----------|----------|
| `HTTP_CONTROL` (기본) | 전원 투입 시 | WiFi WebServer |
| `MQTT_CONTROL` | UWB로 `command/controlled = 1` 수신 시 | UWB (신호등 경유) |

- `MQTT_CONTROL` 모드에서는 웹 UI 조작 불가 (403 반환)
- `command/controlled = 0` 수신 시 `HTTP_CONTROL` 복귀

---

## 통신 프로토콜

### Proto-A (UART: RPi ↔ 신호등)

| 프레이밍 | CRC | CMD 범위 |
|---------|-----|---------|
| STX(0x02) / ETX(0x03) + DLE(0x10) 바이트 스터핑 | CRC-8 (poly=0x07) | 0x50 ~ 0x55 |

주요 커맨드:

| CMD | 값 | 방향 | 설명 |
|-----|-----|------|------|
| `CMD_PING` | 0x50 | RPi → 신호등 | 연결 확인 |
| `CMD_PONG` | 0x51 | 신호등 → RPi | PING 응답 |
| `CMD_RANGE_REPORT` | 0x52 | 신호등 → RPi | UWB 거리 보고 |
| `CMD_CTRL_FWD` | 0x54 | RPi → 신호등 | 차량 제어 릴레이 |

### Proto-B (UWB: 신호등 ↔ 차량)

| 프레이밍 | CRC | 토픽 기반 |
|---------|-----|---------|
| 0x7E / 0x7F | CRC-16 CCITT | 문자열 토픽 |

주요 토픽:

| 토픽 | 방향 | 설명 |
|------|------|------|
| `command/controlled` | 신호등 → 차량 | FSM 모드 전환 (1/0) |
| `command/motor` | 신호등 → 차량 | 속도 설정 |
| `command/led` | 신호등 → 차량 | LED 제어 |

### motor.ino 통신 구조 요약 (실제 구현)

차량 코드 `preCode/motor.ino`는 UWB 수신 프레임을 다음 두 경로로 분기합니다.

1. 토픽 제어 프레임 (0x7E/0x7F + CRC16)
   - `PUBLISH`, `ACK`, QoS, Topic 문자열, Payload 길이 필드를 사용합니다.
   - 구독 토픽: `command/controlled`, `command/motor`, `command/led`, `command/range_probe`
   - 발행 토픽: `telemetry/range/last` (최근 거리 측정값)
2. 거리측정 프레임 (TWR 16바이트)
   - `POLL(0)` → `POLL_ACK(1)` → `RANGE(2)` → `RANGE_REPORT(3)`
   - 차량은 Responder(응답기)로 동작해 거리 계산 후 `RANGE_REPORT`를 송신합니다.

### 거리측정 구현 (RPi ↔ Node1/2/3 ↔ 차량)

현재 구현된 요청/응답은 다음과 같습니다.

1. RPi가 각 노드로 `CMD_RANGE_REQUEST(0x56)` 전송
2. 노드는 UWB Initiator로 차량에 `POLL`/`RANGE` 송신
3. 차량은 UWB Responder로 거리 계산 후 `RANGE_REPORT` 송신
4. 노드는 거리(cm) + RSSI를 `CMD_RANGE_REPORT(0x52)`로 RPi에 반환
5. RPi는 Node1~3의 최신 거리로 2D 삼변측량을 계산해 차량 좌표 출력

`CMD_RANGE_REPORT` payload 형식:
- `[CMD][NODE_ID][DIST_HI][DIST_LO][RSSI_HI][RSSI_LO]`
- 거리 실패 시 `DIST=0xFFFF`

---

## DWM1000 UWB TWR 구현 가이드 (폴링 방식)

> 이 섹션은 DW1000Ng 라이브러리를 **폴링 모드(`initializeNoInterrupt`)** 로 사용하면서  
> TWR(Two-Way Ranging)을 안정적으로 구현하는 과정에서 파악한 정확한 동작 원리를 정리한다.  
> 인터럽트 방식(`initialize()`) 예제와는 구조가 근본적으로 다르므로 주의할 것.

### 1. 폴링 방식 vs 인터럽트 방식 차이

| 항목 | 인터럽트 방식 (`initialize`) | 폴링 방식 (`initializeNoInterrupt`) |
|------|----------------------------|------------------------------------|
| 상태 감지 | `sentAck` / `receivedAck` 콜백 플래그 | `isReceiveDone()` / `isTransmitDone()` loop() 내 직접 조회 |
| RX→TX 전환 | 내부 ISR이 자동 처리 | **`forceTRxOff()` 수동 호출 필수** |
| 대표 예제 | `DWM_EXAMPLE/TwoWayRangingInitiator.ino` | `ESP32/node_car/node_car.ino` |

**핵심**: 폴링 방식에서 RX 수신 후 TX로 전환할 때 `forceTRxOff()`를 호출하지 않으면  
`setTransmitData()` / `startTransmit()` 이후에도 TX가 실제로 실행되지 않는다.

---

### 2. DW1000Ng 라이브러리 레지스터 동작 (소스 분석 결과)

| 함수 | 대상 레지스터 | 동작 설명 |
|------|:------------:|-----------|
| `isReceiveDone()` | SYS_STATUS (0x0F) | 매 호출마다 SPI READ → `_sysstatus` 캐시 갱신 → RXFCG && RXDFR 비트 확인 |
| `getReceivedData()` | RX_BUFFER | SYS_STATUS 무관 — 버퍼 직접 읽기 |
| `getReceiveTimestamp()` | **RX_TIME (0x15)** | **SYS_STATUS와 완전히 독립된 레지스터** — clearReceiveStatus() 이후에도 유효 |
| `clearReceiveStatus()` | SYS_STATUS (0x0F) | RX 완료 비트(RXDFR, RXFCG 등) write-1-to-clear — **RX_TIME 건드리지 않음** |
| `getTransmitTimestamp()` | TX_TIME | SYS_STATUS 무관 — TX 완료 타임스탬프 읽기 |
| `clearTransmitStatus()` | SYS_STATUS (0x0F) | TX 완료 비트만 클리어 |
| `forceTRxOff()` | SYS_CTRL | **SYS_STATUS 건드리지 않음** — 송수신 강제 종료만 |
| `startReceive()` | SYS_CTRL | **SYS_STATUS 건드리지 않음** — RX 모드 진입만 |
| `startTransmit()` | SYS_CTRL | **SYS_STATUS 건드리지 않음** — TX 시작만 |

> **중요**: `getReceiveTimestamp()`와 `clearReceiveStatus()`는 서로 다른 레지스터를 건드리므로  
> 순서는 기술적으로 독립적이다. 그러나 `clearReceiveStatus()`를 TX 함수들보다 **먼저** 호출해야  
> TX 폴링 루프(`isTransmitDone()`) 중 `_sysstatus` 캐시가 오염되어 RX 완료 비트가 남아있는 문제를 방지한다.

---

### 3. processMessages() 올바른 구현 순서

```cpp
void processMessages() {
    if (!DW1000Ng::isReceiveDone()) return;   // (A) SPI READ: SYS_STATUS 확인

    int dataLen = DW1000Ng::getReceivedDataLength();
    DW1000Ng::getReceivedData(tempBuffer, dataLen);    // RX_BUFFER 읽기

    // (B) RX_TIME(0x15) 읽기 — SYS_STATUS와 독립된 레지스터이므로 순서 자유
    uint64_t rxTimestamp = DW1000Ng::getReceiveTimestamp();

    // (C) SYS_STATUS RX 비트 클리어 — TX 폴링 시작 전에 반드시 완료
    //     이 이후 isReceiveDone()은 false를 반환하므로 동일 프레임 이중 처리 방지
    DW1000Ng::clearReceiveStatus();

    // (D) 프레임 처리 (타임스탬프는 파라미터로 전달)
    bool handled = handleRangingFrame(tempBuffer, dataLen, rxTimestamp);
    if (!handled) {
        // MQTT/토픽 처리 로직
        DW1000Ng::startReceive();  // TX 없이 종료 시 RX 복귀 필수
    }
    // Ranging 경로: TX 함수(sendPollAckFrame 등) 내부에서 startReceive() 호출됨
}
```

**주의**: `handleRangingFrame()` 내부에서 `getReceiveTimestamp()`를 재호출하지 말 것.  
타임스탬프는 반드시 `processMessages()`에서 취득 후 파라미터로 전달해야 한다.

---

### 4. TX 완료 후 RX 복귀 패턴 (필수)

모든 TX 함수(`sendPollAckFrame`, `sendRangeReportFrame`, `publish`, `sendAck` 등)는  
마지막에 반드시 `startReceive()`를 호출해야 한다. DWM1000은 **반이중** 장치이므로  
TX 완료 후 명시적으로 RX 모드로 전환하지 않으면 다음 프레임을 수신할 수 없다.

```cpp
void sendPollAckFrame() {
    DW1000Ng::forceTRxOff();          // (1) RX→TX 전환: 폴링 방식 필수
    DW1000Ng::setTransmitData(...);
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);
    // TX 완료 폴링
    uint32_t txStart = millis();
    while (!DW1000Ng::isTransmitDone()) {
        if (millis() - txStart > 100) break;
    }
    DW1000Ng::clearTransmitStatus();
    timePollAckSent = DW1000Ng::getTransmitTimestamp(); // TX_TIME 레지스터
    DW1000Ng::startReceive();         // (2) TX 완료 후 즉시 RX 복귀
}
```

---

### 5. DELAYED TX 구현 (이니시에이터 측 RANGE 프레임)

POLL_ACK 수신 후 RANGE 프레임을 즉시 전송하면 리스폰더가 아직 RX 모드로 전환되기 전이라  
RANGE 프레임을 놓칠 수 있다. `DELAYED TX` + 3000µs 지연으로 해결한다.

```cpp
void send_range() {
    DW1000Ng::forceTRxOff();   // (1) 반드시 setDelayedTRX 이전에 호출

    uint64_t timeRangeSent = DW1000Ng::getSystemTimestamp();
    timeRangeSent += DW1000NgTime::microsecondsToUWBTime(3000); // 3ms 뒤 예약

    byte futureTimeBytes[LENGTH_TIMESTAMP];
    DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
    DW1000Ng::setDelayedTRX(futureTimeBytes);  // (2) forceTRxOff 이후에 유효

    // 실제 TX 시각 = 예약 시각 + 안테나 딜레이
    timeRangeSent += DW1000Ng::getTxAntennaDelay();

    // ★ 이 구간에서 Serial.println 절대 금지
    //   (µs 단위 지연 발생 시 예약 시각 초과 → 즉시 TX 불가)
    buildRangeFrame(uwbFrame);  // 프레임 구성 (빠른 메모리 연산만)

    DW1000Ng::setTransmitData(uwbFrame, UWB_FRAME_LEN);
    DW1000Ng::startTransmit(TransmitMode::DELAYED);  // (3) 예약 시각에 TX
}
```

**핵심**: `setDelayedTRX()` 이후 `startTransmit(DELAYED)` 호출 사이에 `Serial.println` 등  
µs 단위 지연이 발생하는 코드를 넣으면 예약 시각을 초과해 TX가 실패한다.

---

### 6. TWR 프레임 시퀀스 (4-메시지 Two-Way Ranging)

```
이니시에이터 (Node)                      리스폰더 (Car)
─────────────────────────────────────────────────────────
   POLL 전송 (timePollSent)
                         ────────────────>
                                          timePollReceived = rxTimestamp
                                          sendPollAckFrame()
                                          timePollAckSent = TX_TIME
                         <────────────────
   timePollAckReceived = rxTimestamp
   clearReceiveStatus()
   10ms 대기 (PENDING_RANGE_SEND)
   send_range() with DELAYED TX 3000µs
   (timeRangeSent 예약)
                         ────────────────>
                                          timeRangeReceived = rxTimestamp
                                          computeRangeAsymmetric(
                                            timePollSent, timePollReceived,
                                            timePollAckSent, timePollAckReceived,
                                            timeRangeSent, timeRangeReceived)
                                          sendRangeReportFrame(distance_cm)
                         <────────────────
   거리값(cm) 수신 출력
```

**6개 타임스탬프 취득 위치**:

| 타임스탬프 | 취득 노드 | 취득 시점 |
|-----------|---------|---------|
| `timePollSent` | 이니시에이터 | POLL TX 완료 후 `getTransmitTimestamp()` |
| `timePollReceived` | 리스폰더 | POLL 수신 후 `rxTimestamp` 파라미터 |
| `timePollAckSent` | 리스폰더 | POLL_ACK TX 완료 후 `getTransmitTimestamp()` |
| `timePollAckReceived` | 이니시에이터 | POLL_ACK 수신 후 `rxTimestamp` 파라미터 |
| `timeRangeSent` | 이니시에이터 | RANGE DELAYED TX 예약 시각 + 안테나 딜레이 |
| `timeRangeReceived` | 리스폰더 | RANGE 수신 후 `rxTimestamp` 파라미터 |

---

### 7. 버그 이력 및 해결책

#### 버그 1: RANGE 프레임이 리스폰더에 전혀 도달하지 않음
- **원인**: POLL_ACK 수신 직후 IMMEDIATE TX로 RANGE 전송 → 리스폰더가 아직 TX 중이거나  
  RX 모드로 복귀하기 전이라 프레임 손실
- **해결**: `TransmitMode::DELAYED` + 3000µs 지연 적용 (`setDelayedTRX` + `startTransmit(DELAYED)`)

#### 버그 2: sendPollAckFrame() TX가 실행되지 않음 (폴링 방식)
- **원인**: 인터럽트 방식 예제와 달리 폴링 방식에서는 RX 상태에서 TX 전환 시  
  `forceTRxOff()` 수동 호출 없이는 `startTransmit()` 무시됨
- **해결**: `sendPollAckFrame()` 첫 줄에 `DW1000Ng::forceTRxOff()` 추가

#### 버그 3: 동일한 POLL 프레임이 반복 처리됨 (이중 처리)
- **원인**: `handleRangingFrame()` 내부에서 `sendPollAckFrame()` 호출 →  
  TX 완료 폴링(`isTransmitDone()`) 중 `SYS_STATUS` 캐시가 갱신 →  
  이전 RX 완료 비트가 아직 남아 `isReceiveDone()` 재발화
- **해결**: `processMessages()`에서 `clearReceiveStatus()`를 TX 호출보다 **먼저** 실행

#### 버그 4: `RANGE_FAILED` — 거리 계산 결과가 범위를 벗어남 (0 이하 또는 100m 초과)
- **원인**: `getReceiveTimestamp()`를 `forceTRxOff()` 또는 `sendPollAckFrame()` 이후에 호출 →  
  `handleRangingFrame()` 내부에서 타임스탬프 취득 시점이 TX 시작 이후 → 값 오염 가능성
- **해결**: `processMessages()`에서 미리 `rxTimestamp = getReceiveTimestamp()` 취득 후  
  `handleRangingFrame(data, len, rxTimestamp)` 파라미터로 전달

#### 버그 5: 300ms 주기 강제 RX 재초기화가 RANGE 수신을 방해
- **원인**: `millis()` 기반 300ms 타이머로 `forceTRxOff()` + `startReceive()` 강제 실행 →  
  RANGE 프레임 도달 타이밍에 RX가 재초기화되어 프레임 드롭
- **해결**: 해당 타이머 로직 완전 제거. TX 완료 후 `startReceive()` 패턴만으로 충분

---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| `/dev/ttyESP_Node*` 없음 | udev 규칙 미설치 또는 재부팅 안 함 | `sudo bash scripts/setup_uart.sh` 후 재부팅 |
| UART 통신 안 됨 | TX↔RX 교차 미연결 또는 GND 미연결 | 배선 확인 (RPi TX → ESP32 RX, RPi RX ← ESP32 TX) |
| `DW1000Ng` 초기화 실패 | SPI 핀 연결 오류 | SCK=18, MISO=19, MOSI=23, SS=5, RST=22 확인 |
| 차량 WiFi 접속 불가 | softAP 미시작 | 시리얼 모니터에서 WiFi 초기화 로그 확인 |
| UWB 거리 측정 불가 | 안테나 딜레이 미설정 또는 네트워크 ID 불일치 | `setAntennaDelay(16436)`, `setNetworkId(10)` 확인 |
| `MQTT_CONTROL` 모드에서 웹 조작 불가 | 정상 동작 | UWB로 `command/controlled = 0` 전송하여 HTTP 모드 복귀 |

---

## 신호등 노드 (node_signal) 운용 가이드

> `ESP32/node_signal/` — FSM + LED + UWB signal_state + RPi UART 통합 구현

### FSM Phase 표

| Phase | 차량 신호 | 보행자 신호 | 설명 | 지속 시간 |
|:-----:|:--------:|:---------:|------|:--------:|
| 0 | 🟢 GREEN | 🔴 RED | 차량 통행 | 8초 |
| 1 | 🟡 YELLOW | 🔴 RED | 차량 감속 | 2초 |
| 2 | 🔴 RED | 🔴 RED | 전환 대기 | 2초 |
| 3 | 🔴 RED | 🔴 RED | ALL STOP (보행 진입 전) | 1초 |
| 4 | 🔴 RED | 🟢 GREEN | 보행자 통행 | 5초 (+PED_EXTEND 최대 9초) |
| 5 | 🔴 RED | 🔴 RED | 보행 종료 대기 | 2초 |
| 6 | 🔴 RED | 🔴 RED | ALL STOP (차량 진입 전) | 1초 |

---

### ESP32 시리얼 명령어

시리얼 모니터(115200 baud)에서 직접 입력 가능.

| 명령어 | 동작 |
|--------|------|
| `RANGE_TRIGGER` | UWB tracking 시작 — 1초마다 `telemetry/signal_state` 송신 |
| `RANGE_STOP` | UWB tracking 종료 — `telemetry/signal_end` 1회 송신 후 중단 |
| `PED_EXTEND` | 보행 연장 +3초 (phase 4일 때만, 최대 9초) |
| `EMERGENCY` | 비상 모드 전환 — 긴급차량 우선 신호 |
| `STATUS` | 현재 phase / mode / extraWalk / tracking 상태 출력 |

---

### RPi → 신호등 제어 명령어 (command/light)

RPi `uart_test` 프롬프트에서 입력. `N` = 노드 번호 (1 / 2 / 3).

| RPi 입력 | 동작 |
|----------|------|
| `CMD N command/light C9` | Node N EMERGENCY 모드 |
| `CMD N command/light C1` | Node N 차량 GREEN 강제 (phase 0) |
| `CMD N command/light C0` | Node N 차량 RED 강제 (phase 2) |
| `CMD N command/light P1` | Node N 보행 연장 +3초 (PED_EXTEND) |
| `CMD N command/light P0` | Node N 보행 연장 취소 |
| `CMD command/light C9` | 전체 노드 EMERGENCY |
| `PING N` | Node N만 PING |
| `PING` | 전체 노드 PING |
| `RANGE N` | Node N만 거리 측정 |
| `RANGE` | 전체 노드 거리 측정 |

**예시:**
```
CMD 1 command/light C9      → Node1만 emergency
CMD 2 command/light P1      → Node2만 보행 연장
CMD command/light C0        → 전체 차량 RED 강제
PING 1                      → Node1 생존 확인
```

---

### UWB signal_state 토픽

신호등 → 차량으로 1초마다 송신 (`telemetry/signal_state`).

**payload 구조: 2바이트 `[stateByte, remainSec]`**

| 비트 | 필드 | 값 |
|:----:|------|----|
| bit 0-1 | 차량 신호 | 0=GREEN / 1=YELLOW / 2=RED |
| bit 2-3 | 보행자 신호 | 0=WALK / 1=RED |
| bit 4 | pedExtend | 1=연장 중 |
| bit 5 | emergency | 1=비상 모드 |

**예시:**
```
0x04 = 0b00000100 → 차량 RED(2), 보행자 WALK(0), 연장 없음, 비상 없음
0x14 = 0b00010100 → 차량 RED(2), 보행자 WALK(0), 연장 중
```

`telemetry/signal_end` — 차량이 threshold 벗어날 때 1회 송신 (세션 종료 알림).

---

### LED 핀 배치 (node_signal)

| LED | ESP32 GPIO |
|:---:|:---------:|
| 차량 RED | 13 |
| 차량 YELLOW | 14 |
| 차량 GREEN | 15 |
| 보행자 RED | 32 |
| 보행자 GREEN | 33 |
---

## 참고 문서

- [CLAUDE.md](CLAUDE.md) — 프로젝트 헌장 (AI 코딩 규칙 전문)
- [preCode/motor.ino](preCode/motor.ino) — 차량 ESP32 참조 코드 (이전 프로젝트)
- [preCode/mqtt_esp32.ino](preCode/mqtt_esp32.ino) — UART+UWB 통신 참조 코드
- [DW1000Ng 라이브러리](https://github.com/F-Army/DW1000Ng) — DWM1000 Arduino 라이브러리

---

## 라이선스

이 프로젝트는 학습/연구 목적으로 작성되었습니다.
