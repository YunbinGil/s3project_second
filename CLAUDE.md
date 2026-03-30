# CLAUDE.md — 프로젝트 헌장: UWB 삼변측량 기반 차량 위치 추적 시스템

> 라즈베리파이(중앙 모니터) ↔ 신호등 ESP32 ×3 (UART) ↔ 차량 ESP32 ×1 (UWB)

---

## 1. 워크플로우 강제 (Meta-Instructions)

- **계획 후 실행 원칙**: 코드를 즉시 생성하지 말 것. 반드시 다음 순서를 따를 것:
  1. 기존 파일 구조 및 통신 로직 분석
  2. 수정할 파일, 사용할 API, 아키텍처 변경 계획을 텍스트로 출력
  3. 사용자 승인 후 코드 작성
- **심화 규칙 참조**: 특정 도메인 작업 시 아래 **섹션 6의 파일 포인터**를 먼저 읽고 구현할 것.
- **ESP32 빌드 명령**: Arduino IDE(로컬 PC) 기준으로 제시. Remote SSH 터미널에서 ESP32 빌드/플래시 명령 제안 금지.
- **환경 전제 금지**: x86 범용 리눅스나 Windows 환경을 절대 가정하지 말 것.
  - RPi 호스트: **Linux C++17 + POSIX API**
  - ESP32: **Arduino 프레임워크** (ESP-IDF 직접 사용 금지, `idf.py` 명령 제안 금지)

### 터미널 컨텍스트 규칙 (이중 빌드 환경)

| 작업 도메인 | 사용 터미널 | 비고 |
|------------|------------|------|
| `raspberryPI/` 빌드·실행·디버그 | **VS Code Remote SSH 터미널** (Raspberry Pi) | cmake, gcc-12, 리눅스 시스템 API |
| `ESP32/` 빌드·업로드·모니터 | **Arduino IDE (로컬 PC)** | 스케치 컴파일·업로드, 시리얼 모니터 |

- `raspberryPI/` 코드 수정 시 → Remote SSH 터미널 기준으로 경로·명령어 제시
- `ESP32/` 코드 수정 시 → Arduino IDE 기준으로 `.ino` 스케치 및 `.h/.cpp` 파일 작성
- **두 환경을 절대 혼용하지 말 것** (예: RPi 터미널에서 ESP32 빌드 시도 금지)

---

## 2. 하드웨어 토폴로지 및 런타임 환경

### 시스템 개요

| 구분 | 사양 |
|------|------|
| **호스트** | Raspberry Pi 4/5, Linux (Remote SSH), C++17 |
| **신호등 노드** | ESP32 × 3대 (Arduino), 각각 DWM1000 탑재, UART로 RPi 연결 |
| **차량 노드** | ESP32 × 1대 (Arduino), DWM1000 + 모터(TB6612FNG) + WiFi AP, **RPi 미연결** |
| **RPi ↔ 신호등** | PL011 UART × 3채널 (점대점, 비동기 직렬, 115200 baud) |
| **신호등 ↔ 차량** | DWM1000 UWB (IEEE 802.15.4a, 무선) |
| **RPi 빌드 터미널** | VS Code Remote SSH 터미널 (`cmake` + `make`) |
| **ESP32 빌드·업로드** | Arduino IDE (로컬 PC, 보드: ESP32 Dev Module) |
| **프로젝트 목적** | **UWB 삼변측량 기반 차량 실시간 위치 추적 + 신호등 자동 제어** |

### 주변기기 사양

| 모듈 | 칩셋 | 인터페이스 | Arduino 라이브러리 | 탑재 위치 |
|------|------|----------|------------------|----------|
| DWM1000 | DW1000 | SPI (4-wire + RST + IRQ) | `DW1000Ng` (F-Army) | 신호등 ×3 + 차량 ×1 |
| TB6612FNG | TB6612FNG | GPIO(AIN1,AIN2,STBY) + MCPWM(PWMA) | ESP32 내장 MCPWM API | 차량만 |
| WiFi (내장) | ESP32 내장 | softAP | `WiFi.h` + `WebServer.h` | 차량만 |

### 데이터 흐름 다이어그램

```
              차량 ESP32 (독립, RPi 미연결)
              [DWM1000 + Motor + WiFi AP]
                 /       |        \
           UWB  /   UWB  |   UWB   \   (거리 측정 + 제어 명령)
              /          |           \
  신호등 ESP32-1   신호등 ESP32-2   신호등 ESP32-3
  [DWM1000]        [DWM1000]        [DWM1000]
       |                |                |
   UART(PL011)     UART(PL011)     UART(PL011)
       |                |                |
       └────────  Raspberry Pi  ─────────┘
               (중앙 위치 계산기 / 모니터)
```

**1. 위치 추적 (핵심 데이터 경로):**
```
차량 DWM1000 ←→ 신호등 DWM1000 ×3  (UWB 거리 측정)
→ 신호등 ESP32 → UART → RPi        (CMD_RANGE_REPORT 거리 보고)
→ RPi 에서 삼변측량 계산            → 차량 (x, y) 좌표 출력
```

**2. 차량 제어 (자동 — 신호등 경유):**
```
신호등 ESP32 → DWM1000 UWB → 차량 ESP32
(command/controlled, command/motor 토픽)
→ 차량 FSM → MQTT_CONTROL 모드 → 모터 속도 변경
```

**3. 차량 제어 (수동 — WiFi 직접):**
```
사용자 스마트폰 → WiFi AP → 차량 ESP32 WebServer
→ HTTP_CONTROL 모드 → 모터 속도 변경
```

### UART 핀 매핑 (Raspberry Pi)

`/boot/firmware/config.txt`에 아래 오버레이를 추가하여 UART 활성화:
```
dtoverlay=uart2
dtoverlay=uart3
dtoverlay=uart4
```

| UART | TX (BCM GPIO) | RX (BCM GPIO) | ALT 모드 | 충돌 주의 |
|------|--------------|--------------|---------|---------|
| UART2 | GPIO 4 | GPIO 5 | ALT4 | GPIO 0/1은 HAT EEPROM 예약 — **절대 사용 금지** |
| UART3 | GPIO 8 | GPIO 9 | ALT4 | SPI0(CE0/MISO)과 핀 충돌 주의 |
| UART4 | GPIO 12 | GPIO 13 | ALT4 | PWM0/오디오 출력과 핀 공유 |

> **udev 규칙 필수**: `/dev/ttyAMA*` 번호는 부팅마다 비결정적으로 변경됨.  
> 소스 코드에 `/dev/ttyAMA1` 등을 **절대 하드코딩하지 말 것**.  
> 물리 주소 기반 udev 심볼릭 링크(`/dev/ttyESP_Node1` 등)를 반드시 사용할 것.

---

## 3. 라즈베리파이 (Linux 호스트) 코딩 규칙

### ✅ 필수 구현

- **epoll 단일 스레드 이벤트 루프**: UART 3채널을 하나의 스레드로 다중화
  ```
  epoll_create1(0) → epoll_ctl(EPOLL_CTL_ADD, EPOLLIN | EPOLLERR) × 3
  → epoll_wait(fd, events, MAX, -1) 무한 루프
  ```
- **epoll 트리거 모드**: 레벨 트리거(기본값) 사용. `EPOLLET` 엣지 트리거 **사용 금지**.
- **포트 오픈 플래그**: 반드시 `O_RDWR | O_NOCTTY | O_NONBLOCK` 조합 사용
- **termios 초기화**: `cfmakeraw()` 호출로 Raw 모드 전환 필수
  - `c_cc[VMIN] = 0`, `c_cc[VTIME] = 0` 설정
  - `ICANON`, `ECHO`, `IXON`, `ISIG` 플래그 반드시 소거
- **삼변측량 계산**: 3개 신호등에서 `CMD_RANGE_REPORT` 수집 후 차량 좌표 산출 (RPi 전담)

### ❌ 절대 금지

- `std::thread` 또는 `pthread`로 포트별 개별 스레드 생성 (스레드당 1포트 패턴)
- `read()`를 블로킹 모드로 루프 내에서 직접 호출
- UART 디바이스 경로 (`/dev/ttyAMA*`) 소스 코드 하드코딩
- `select()` 또는 `poll()` 사용 (epoll로 대체)

---

## 4. ESP32 (Arduino 프레임워크) 코딩 규칙

### 4.1 공통 규칙 (모든 ESP32 노드)

- **보드 설정**: Arduino IDE → 보드 매니저 `esp32 by Espressif`, 보드: `ESP32 Dev Module`
- **FSM 파서**: 수신 파서는 반드시 유한 상태 머신(FSM)으로 구현 (단순 indexOf 금지)
- `Serial`(UART0) = **USB 디버그 전용**. 라즈베리파이/외부 통신용 절대 금지.
- `loop()` 안에서 `delay()` 호출 절대 금지 (수신 FSM 지연 발생)
- ESP-IDF 직접 API (`uart_driver_install`, `uart_param_config` 등) 사용 금지
- `idf.py` 빌드 명령 제안 금지

### 4.2 신호등 노드 (UART + DWM1000, ×3, RPi 연결)

**역할**: RPi와 UART 통신 + DWM1000으로 차량과 UWB 거리 측정 **동시 수행**

**핀 배치**: DWM1000 SPI = SS(GPIO5)/RST(GPIO22), UART2 = RXD2(GPIO16)/TXD2(GPIO17)

| GPIO | 기능 | 비고 |
|------|------|------|
| 5 | DWM1000 SS (CS) | SPI 제어 |
| 22 | DWM1000 RST | SPI 제어 |
| 18 | DWM1000 SCK | SPI 클록 |
| 19 | DWM1000 MISO | SPI 데이터 |
| 23 | DWM1000 MOSI | SPI 데이터 |
| 4 | DWM1000 IRQ | (initializeNoInterrupt 시 미사용 가능) |
| **16** | **UART2 RX** (RXD2) | RPi TX에 연결 |
| **17** | **UART2 TX** (TXD2) | RPi RX에 연결 |
| 2 | 내장 LED | 디버그 |

**#define 블록 (필수):**
```cpp
// 신호등 노드 — UART2 사용 (RXD2=GPIO16, TXD2=GPIO17)
#define NODE_ID      1        // 1, 2, 3 중 노드에 맞게 수정
#define UART_TX_PIN  17       // TXD2 (GPIO17) → RPi RX에 연결
#define UART_RX_PIN  16       // RXD2 (GPIO16) → RPi TX에 연결
#define UART_BAUD    115200
```

**UART 수신 처리:**
```cpp
HardwareSerial MySerial(1);  // UART1
MySerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

void loop() {
    while (MySerial.available()) {
        uint8_t b = MySerial.read();
        // FSM 파서에 바이트 단위 투입
    }
}
```

**신호등 loop() 구조:**
```cpp
void loop() {
    uartComm.process();       // 1. UART 수신 (RPi → 신호등)
    uwbComm.process();        // 2. UWB 수신 (차량 → 신호등)
    reportRange();            // 3. 주기적 거리 보고 (millis 기반, UART → RPi)
}
```

### 4.3 차량 노드 (Motor + WiFi + DWM1000, ×1, RPi 미연결)

**역할**: DWM1000으로 신호등과 UWB 통신 + WiFi AP 수동 제어 + 모터 구동

| GPIO | 기능 | 비고 |
|------|------|------|
| 16 | DWM1000 RST | SPI 제어 |
| 17 | DWM1000 SS (CS) | SPI 제어 |
| 18 | DWM1000 SCK | SPI 클록 |
| 19 | DWM1000 MISO | SPI 데이터 |
| 23 | DWM1000 MOSI | SPI 데이터 |
| 4 | DWM1000 IRQ | (선택적) |
| 15 | TB6612 PWMA | MCPWM 출력 |
| 27 | TB6612 AIN1 | 모터 방향 비트 1 |
| 13 | TB6612 AIN2 | 모터 방향 비트 2 |
| 14 | TB6612 STBY | 모터 스탠바이 |
| 2 | 내장 LED | 상태 표시 |

→ 차량은 RPi UART를 사용하지 않으므로 UART 핀 충돌 이슈 없음.

**차량 loop() 구조:**
```cpp
void loop() {
    uwbComm.process();                // 1. UWB 수신 (command/controlled → FSM)
    server.handleClient();            // 2. HTTP 클라이언트 처리
    updateMotorSpeed(currentDirection); // 3. 모터 가감속 (20ms Non-blocking)
}
```

---

## 5. 통신 프로토콜 설계 규칙

### 직렬화 / 역직렬화

- **`#pragma pack` 및 `__attribute__((packed))` 사용 엄격히 금지**  
  → ARM Cortex-A ↔ Xtensa LX6/RISC-V 간 메모리 정렬 및 엔디안 불일치로 SIGBUS 발생 위험
- 다중 바이트 정수는 **반드시 명시적 비트 시프트 연산**으로 직렬화:
  ```c
  // 송신 (빅 엔디안 변환)
  buf[0] = (value >> 8) & 0xFF;
  buf[1] = (value) & 0xFF;
  // 수신 (재조립)
  value = ((uint16_t)buf[0] << 8) | buf[1];
  ```

### 프레이밍

- STX(시작) / ETX(종료) 구분자 + 페이로드 길이 헤더 포함
- 페이로드 내부에 STX/ETX 바이트가 포함될 경우 **바이트 스터핑(DLE 삽입)** 또는 **COBS** 알고리즘 적용
- 수신 파서는 반드시 **유한 상태 머신(FSM)** 으로 구현

### 무결성 검증 (CRC-8)

- UART 프로토콜(Proto-A) 패킷 꼬리에 CRC-8 바이트 부착 필수
- 다항식: `0x07` (CCITT) 또는 `0x8C` (Dallas/Maxim 반사형)
- **반드시 256바이트 룩업 테이블(Table-driven) 방식으로 구현** (동적 비트 루프 방식 금지)

### Proto-A (UART) CMD 코드 정의

| CMD | 값 | 방향 | 페이로드 | 설명 |
|-----|-----|------|---------|------|
| CMD_PING | 0x50 | RPi→신호등 | `[SEQ_HI][SEQ_LO]` | 연결 확인 |
| CMD_PONG | 0x51 | 신호등→RPi | `[SEQ_HI][SEQ_LO][NODE_ID]` | PING 응답 |
| CMD_RANGE_REPORT | 0x52 | 신호등→RPi | `[NODE_ID][DIST_HI][DIST_LO][RSSI_HI][RSSI_LO]` | UWB 거리 측정 결과 |
| CMD_POSITION | 0x53 | RPi→신호등 (선택) | `[X_HI][X_LO][Y_HI][Y_LO]` | RPi 계산 차량 좌표 브로드캐스트 |
| CMD_CTRL_FWD | 0x54 | RPi→신호등 | `[UWB_TOPIC_ID][페이로드...]` | RPi 경유 차량 제어 (신호등이 UWB 릴레이) |
| CMD_MODE_NOTIFY | 0x55 | 신호등→RPi | `[NODE_ID][MODE]` | 차량 FSM 모드 전환 알림 (UWB 수신 후 릴레이) |

거리 직렬화 (빅 엔디안, cm 단위 정수):
```c
DIST_HI = (distance_cm >> 8) & 0xFF;
DIST_LO = (distance_cm)      & 0xFF;
RSSI_HI = (rssi_raw >> 8)    & 0xFF;
RSSI_LO = (rssi_raw)         & 0xFF;
```

### 구현 반영 메모 (2026-03)

현재 코드 구현에서 거리측정 트리거는 다음을 사용한다.

- `CMD_RANGE_REQUEST (0x56)` : `RPi -> Node`
  - payload: `[SEQ_HI][SEQ_LO]`
- `CMD_RANGE_REPORT (0x52)` : `Node -> RPi`
  - payload: `[NODE_ID][DIST_HI][DIST_LO][RSSI_HI][RSSI_LO]`
  - `DIST=0xFFFF`는 측정 실패 의미

역할 분담:
- `preCode/motor.ino` : UWB Responder + 기존 topic 제어(`command/*`) 유지
- `ESP32/node_template/*` : UART 수신 + UWB Initiator + RPi 보고
- `raspberryPI/src/main.cpp` : 요청 송신 + 리포트 수집 + 삼변측량

---

## 6. 심화 규칙 파일 포인터 (Progressive Disclosure)

| 작업 도메인 | 참조 파일 | 트리거 |
|------------|----------|--------|
| 패킷 파서 / FSM / 직렬화 유틸리티 | `.claude/rules/framing-protocol.md` | 프로토콜 파싱 코드 작성 시 |
| CRC-8 알고리즘 및 룩업 테이블 | `.claude/rules/crc-tables.md` | CRC 검증 로직 구현 시 |
| Arduino IDE 보드 설정 / 라이브러리 | `.claude/rules/build-commands.md` | Arduino IDE 보드·포트 설정 변경 시 |
| GPIO 핀 번호 및 UART 매핑 상세 | `.claude/rules/pin-mappings.md` | `config.txt` 또는 핀 설정 수정 시 |
| DWM1000 설정 및 UWB 통신 | `.claude/rules/uwb-dwm1000.md` | DWM1000 초기화/수신/송신 코드 작성 시 |
| UWB 위치 추적 / 삼변측량 | `.claude/rules/trilateration.md` | 거리 보고/좌표 계산 코드 작성 시 |
| FSM 제어 모드 패턴 | `.claude/rules/fsm-control-mode.md` | 모드 전환 로직 구현 시 |
| 모터 제어 (MCPWM + TB6612FNG) | `.claude/rules/motor-control.md` | MCPWM/모터 코드 작성 시 |
| WiFi AP + WebServer | `.claude/rules/wifi-webserver.md` | HTTP 핸들러 구현 시 |

---

## 7. 프로젝트 디렉토리 구조

```
s3project/
├── CLAUDE.md
├── .github/
│   └── copilot-instructions.md
├── .vscode/
│   └── settings.json
├── preCode/                   # 이전 프로젝트 참조 코드 (수정 금지)
│   ├── motor.ino              # 차량 ESP32 원본 (DWM1000+Motor+WiFi)
│   └── mqtt_esp32.ino         # UART+UWB 통신 원본
│
├── raspberryPI/               # ← Remote SSH 터미널에서 빌드
│   ├── CMakeLists.txt
│   ├── scripts/
│   │   └── setup_uart.sh      # UART overlay + udev 설치
│   ├── udev/
│   │   └── 99-esp32-uart.rules
│   └── src/
│       ├── main.cpp           # epoll 루프 + 삼변측량 계산
│       ├── uart_manager.h/cpp # epoll 이벤트 루프
│       └── protocol.h/cpp     # CRC-8 룩업 테이블 / FSM
│
└── ESP32/                     # ← Arduino IDE (로컬 PC) 에서 빌드·업로드
    ├── node_template/         # 신호등 노드 ×3 공통 스케치
    │   ├── node_template.ino  # setup/loop (UART + DWM1000)
    │   ├── uart_comm.h/cpp    # HardwareSerial UART1 래퍼 (RXD2=GPIO16, TXD2=GPIO17)
    │   ├── uwb_comm.h/cpp     # DWM1000 UWB 송수신 래퍼
    │   ├── protocol.h/cpp     # CRC-8 FSM 파서 (UART 프로토콜)
    │   └── uwb_protocol.h/cpp # CRC-16 토픽 파서 (UWB 프로토콜)
    │
    └── node_car/              # 차량 노드 ×1 독립 스케치 (RPi 미연결)
        ├── node_car.ino       # setup/loop
        ├── uwb_comm.h/cpp     # DWM1000 UWB 수신 래퍼
        ├── uwb_protocol.h/cpp # CRC-16 토픽 파서
        ├── motor_ctrl.h/cpp   # MCPWM + TB6612FNG
        ├── web_server.h/cpp   # WiFi AP + WebServer
        └── fsm_mode.h/cpp     # FSM 제어 모드 (HTTP/MQTT)
```

> **Arduino IDE 제약**: `.ino`와 같은 폴더 내 `.h/.cpp`만 자동 컴파일됨.  
> `node_template/`과 `node_car/`에서 공통 파일(`uwb_protocol.h/cpp` 등)은 **복사하여 사용**.  
> 원본 수정 시 양쪽 폴더에 동기화 필수.

### 신호등 ESP32 업로드 절차 (Arduino IDE, 로컬 PC)

```
1. Arduino IDE → 파일 → 열기 → ESP32/node_template/node_template.ino
2. uart_comm.h 상단의 NODE_ID (1/2/3) 수정 (UART_TX_PIN=17, UART_RX_PIN=16 고정)
3. 도구 → 보드 → esp32 by Espressif → ESP32 Dev Module
4. 도구 → 포트 → ESP32 연결된 COM 포트 선택
5. 스케치 → 업로드 (Ctrl+U)
6. 3대 반복 (NODE_ID만 변경)
```

### 차량 ESP32 업로드 절차 (Arduino IDE, 로컬 PC)

```
1. Arduino IDE → 파일 → 열기 → ESP32/node_car/node_car.ino
2. 도구 → 보드 → esp32 by Espressif → ESP32 Dev Module
3. 도구 → 포트 → 차량 ESP32 연결된 COM 포트 선택
4. 스케치 → 업로드 (Ctrl+U)
```

### RPi 빌드 명령어 (Remote SSH 터미널)

```bash
cd raspberryPI
sudo bash scripts/setup_uart.sh   # 최초 1회 (재부팅 필요)
mkdir -p build && cd build
cmake .. && make -j$(nproc)
./uart_test
```

---

## 8. DWM1000 / UWB 통신 규칙

### 하드웨어 사양

| 항목 | 값 |
|------|-----|
| 모듈 | Decawave DWM1000 |
| 통신 표준 | IEEE 802.15.4a UWB |
| SPI 핀 | SCK=18, MISO=19, MOSI=23, SS=5, RST=22, IRQ=4 |
| Arduino 라이브러리 | `DW1000Ng` (by F-Army) |
| 채널 | Channel 5 |
| 데이터 레이트 | 6.8 Mbps |
| 프리앰블 | LEN_128 (고속 모드) |
| SFD | STANDARD_SFD |
| 탑재 대상 | 신호등 ESP32 ×3 + 차량 ESP32 ×1 (총 4대) |

### DWM1000 네트워크 식별

| 노드 | Device Address | Network ID | 역할 |
|------|---------------|------------|------|
| 신호등 1 | 1 | 10 | UWB 앵커 (고정 위치) |
| 신호등 2 | 2 | 10 | UWB 앵커 (고정 위치) |
| 신호등 3 | 3 | 10 | UWB 앵커 (고정 위치) |
| 차량 | 6 | 10 | UWB 태그 (이동체) |

### DWM1000 기본 설정 코드 (Arduino)

```cpp
device_configuration_t DEFAULT_CONFIG = {
    false, true, true, true, false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_6800KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_128,
    PreambleCode::CODE_3
};

SPI.begin(18, 19, 23, 5);              // SCK, MISO, MOSI, SS
DW1000Ng::initializeNoInterrupt(5, 22);  // SS, RST
DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
DW1000Ng::setDeviceAddress(NODE_ADDR);
DW1000Ng::setNetworkId(10);
DW1000Ng::setAntennaDelay(16436);
DW1000Ng::startReceive();
```

### ✅ 필수 구현

- `SPI.begin(SCK, MISO, MOSI, SS)` → `DW1000Ng::initializeNoInterrupt(SS, RST)`
- 수신: `loop()` 내에서 `DW1000Ng::isReceiveDone()` 폴링 (Non-blocking)
  - 완료 시 → `getReceivedData()` → `parseMessage()` → `clearReceiveStatus()` → `startReceive()`
- 송신: `DW1000Ng::setTransmitData()` → `startTransmit()`
  - 완료 대기 → `clearTransmitStatus()` → `startReceive()` (수신 모드 복귀)
- `setAntennaDelay()` 캘리브레이션 값 반드시 적용

### ❌ 절대 금지

- `DW1000Ng::initialize()` (인터럽트 버전) — `initializeNoInterrupt()` 사용
- 수신 루프에서 blocking wait 금지
- DWM1000 SPI 핀(16,17,18,19,23)을 UART1에 동시 할당 금지
- UWB 송신 후 `startReceive()` 미호출 (반이중이므로 수신 모드 복귀 필수)

---

## 9. UWB 위치 추적 (Trilateration) 규칙

### 아키텍처

- **신호등 ESP32 ×3** = UWB 앵커 (고정 좌표, 사전 설정)
- **차량 ESP32 ×1** = UWB 태그 (이동체, 좌표 미지)
- 각 신호등이 차량과의 UWB 거리를 측정 → UART로 RPi에 `CMD_RANGE_REPORT(0x52)` 전송
- RPi가 3개 거리값 수집 → **삼변측량** → 차량 `(x, y)` 좌표 계산

### 신호등 측 (ESP32)

- UWB 레인징(TWR: Two-Way Ranging) 결과에서 거리(cm) 추출
- 거리값을 `uint16_t` (빅 엔디안)으로 직렬화하여 UART `CMD_RANGE_REPORT` 패킷으로 RPi 전송
- 측정 주기: `millis()` 기반 Non-blocking 타이머 (예: 100ms 간격)

### RPi 측 (C++)

- 3개 채널에서 `CMD_RANGE_REPORT` 수집
- 신호등 고정 좌표 `(x1,y1)`, `(x2,y2)`, `(x3,y3)` 는 설정 파일 또는 argv 입력
- 삼변측량: 3개 거리 `d1, d2, d3` + 앵커 좌표로 연립방정식 풀이 (최소자승법 또는 해석적 해)
- 계산된 차량 좌표를 콘솔/로그로 출력

### ✅ 필수 구현

- 거리 직렬화: `uint16_t` (cm 단위, 빅 엔디안 비트 시프트)
- 측정 주기 타이머: `millis()` 기반 (`delay()` 금지)
- RPi에서 3개 거리가 모두 갱신된 시점에 좌표 계산

### ❌ 절대 금지

- 부동소수점을 UART로 직접 전송 (반드시 정수 변환 후 비트 시프트 직렬화)
- 신호등 ESP32에서 삼변측량 계산 시도 (RPi 역할)
- 거리 측정 주기에 `delay()` 사용

---

## 10. FSM 제어 모드 설계 규칙 (차량 노드 전용)

### 제어 모드 정의

| 모드 | 입력 소스 | 설명 |
|------|----------|------|
| `HTTP_CONTROL` (기본값) | WiFi WebServer | 사용자가 웹 UI로 차량 직접 조작 |
| `MQTT_CONTROL` | DWM1000 UWB | 신호등이 UWB로 전달한 자동 제어 명령 |

### 상태 전이 규칙

- `command/controlled = 1` 수신 → `MQTT_CONTROL` 활성화 (HTTP 입력 차단)
- `command/controlled = 0` 수신 → `HTTP_CONTROL` 복귀 (HTTP 입력 허용)
- 전이 트리거는 **UWB 수신 경로에서만** 발생 (HTTP에서 모드 전환 불가)

### ✅ 필수 구현

- **모드 게이팅 함수:**
  ```cpp
  void setTargetSpeed(float speed, ControlMode requiredMode) {
      if (controlMode == requiredMode) {
          targetSpeed = speed;
      }
      // 모드 불일치 시 무시 (race condition 차단)
  }
  ```
- HTTP 핸들러: `controlMode == HTTP_CONTROL` 검사 후 동작
  - `MQTT_CONTROL` 상태에서 HTTP 요청 → 403 반환
- MQTT 핸들러: `controlMode == MQTT_CONTROL` 검사 후 동작
  - `HTTP_CONTROL` 상태에서 MQTT 명령 → 무시

### ❌ 절대 금지

- HTTP ↔ MQTT 모드 동시 활성화
- `controlMode`를 EEPROM/NVS 영구 저장 (전원 복구 시 항상 `HTTP_CONTROL`)
- `targetSpeed`를 모드 검사 없이 직접 대입
- FSM 전이를 HTTP 경로에서 트리거

---

## 11. 모터 제어 (MCPWM + TB6612FNG) 규칙 — 차량 노드 전용

### 하드웨어 연결

| TB6612FNG 핀 | ESP32 GPIO | 기능 |
|-------------|-----------|------|
| PWMA | GPIO 15 | 속도 (PWM duty 0~100%) |
| AIN1 | GPIO 27 | 방향 비트 1 |
| AIN2 | GPIO 13 | 방향 비트 2 |
| STBY | GPIO 14 | 스탠바이 해제 (HIGH=활성) |

### ✅ 필수 구현

- MCPWM 초기화: `mcpwm_gpio_init()` → `mcpwm_init()` (UP_DOWN_COUNTER, 20kHz)
- 방향: `gpio_set_level()`로 AIN1/AIN2 제어
  - 정회전: AIN1=1, AIN2=0 / 역회전: AIN1=0, AIN2=1 / 정지: AIN1=0, AIN2=0
- 가감속: `targetSpeed`까지 20ms 간격 × 2.5% 단위 점진 변화 (`millis()` 기반 Non-blocking)
- 속도 범위: 0.0 ~ 100.0 (`mcpwm_set_duty` 퍼센트 직접 대응)
- STBY 핀: `setup()`에서 HIGH로 설정

### ❌ 절대 금지

- `updateMotorSpeed()`에서 `delay()` 사용
- `targetSpeed` 직접 대입 (반드시 `setTargetSpeed(speed, mode)` 경유)
- MCPWM 대신 `analogWrite()` / `ledcWrite()` 사용
- GPIO 직접 제어로 PWM 생성 시도

---

## 12. WiFi AP + WebServer 규칙 — 차량 노드 전용

### ✅ 필수 구현

- `WiFi.softAP(ssid, password)` — 비밀번호 8자 이상, SSID/PW는 `#define` 분리
- `WebServer server(80)` — 기본 IP: `192.168.4.1`
- `loop()`에서 `server.handleClient()` 호출 (Non-blocking)
- REST 엔드포인트:

| 경로 | 기능 |
|------|------|
| `/` | 제어 UI HTML 제공 |
| `/up` | 속도 +10 (`HTTP_CONTROL` 모드에서만) |
| `/down` | 속도 -10 (`HTTP_CONTROL` 모드에서만) |

- 모든 핸들러에서 `controlMode == HTTP_CONTROL` 검사 필수
  - `MQTT_CONTROL` 상태 → `server.send(403, ...)` 반환

### ❌ 절대 금지

- WiFi/WebServer 관련 `delay()` 사용
- HTTP 핸들러에서 `controlMode` 검사 우회하여 `targetSpeed` 직접 변경

---

## 13. 이중 프로토콜 체계 (UART vs UWB)

### 프로토콜 레이어 분리

| 프로토콜 | 구간 | 프레이밍 | CRC | 용도 |
|---------|------|---------|-----|------|
| **Proto-A** (UART) | RPi ↔ 신호등 ESP32 | STX(0x02)/ETX(0x03) + DLE(0x10) | CRC-8 (poly=0x07) | 거리 보고, RPi 명령 |
| **Proto-B** (UWB) | 신호등 ↔ 차량 ESP32 | 0x7E / 0x7F | CRC-16 CCITT | 제어 명령, 모드 전환 |

### 규칙

- 프레이밍 바이트가 다르므로 (0x02/0x03 vs 0x7E/0x7F) 혼선 없음
- 각 프로토콜은 **별도 파서 인스턴스**로 처리
  - `ProtoParser` (CRC-8) → UART 전용 (신호등)
  - `UwbParser` (CRC-16) → DWM1000 전용 (신호등 + 차량)
- **크로스 레이어 라우팅** 가능:
  - RPi → UART → 신호등 → UWB → 차량 (RPi 경유 차량 제어)
  - 차량 ← UWB ← 신호등 ← UART ← RPi (역경로)

---

## 14. 토픽/커맨드 체계 (Command Taxonomy)

### Proto-A (UART, RPi ↔ 신호등) — CMD 코드 0x50~0x5F

| CMD | 값 | 방향 | 페이로드 |
|-----|-----|------|---------|
| CMD_PING | 0x50 | RPi→신호등 | `[SEQ_HI][SEQ_LO]` |
| CMD_PONG | 0x51 | 신호등→RPi | `[SEQ_HI][SEQ_LO][NODE_ID]` |
| CMD_RANGE_REPORT | 0x52 | 신호등→RPi | `[NODE_ID][DIST_HI][DIST_LO][RSSI_HI][RSSI_LO]` |
| CMD_POSITION | 0x53 | RPi→신호등 | `[X_HI][X_LO][Y_HI][Y_LO]` |
| CMD_CTRL_FWD | 0x54 | RPi→신호등 | `[UWB_TOPIC_ID][페이로드...]` |
| CMD_MODE_NOTIFY | 0x55 | 신호등→RPi | `[NODE_ID][MODE]` |

### Proto-B (UWB, 신호등 ↔ 차량) — 토픽 문자열 기반 (preCode 호환)

| 토픽 | 페이로드 | 방향 | 설명 |
|------|---------|------|------|
| `command/controlled` | `'1'` 또는 `'0'` | 신호등→차량 | FSM 모드 전환 |
| `command/motor` | float 문자열 (예: `"50.0"`) | 신호등→차량 | 속도 설정 |
| `command/led` | `'1'` 또는 `'0'` | 신호등→차량 | LED 제어 |
| `esp32/status` | 바이너리 | 차량→신호등 | 상태 보고 |

---

## 15. 절대 금지 규칙 종합

| 금지 항목 | 이유 |
|-----------|------|
| `std::thread` / `pthread` 포트별 분리 | epoll 단일 스레드 루프로 대체 |
| `/dev/ttyAMA*` 하드코딩 | 부팅마다 비결정적 변경 — udev 심볼릭 링크 사용 |
| `#pragma pack` / `__attribute__((packed))` | 이기종 아키텍처 간 SIGBUS 발생 위험 |
| ESP32에서 `Serial` (UART0) 통신 사용 | UART0는 USB 디버그 전용 — `HardwareSerial(1)` 사용 |
| `loop()` 안에서 `delay()` 호출 | 수신 FSM 처리 지연 발생 |
| `EPOLLET` 엣지 트리거 | UART에서 불안정 — 레벨 트리거만 사용 |
| `select()` / `poll()` | epoll로 대체 |
| ESP-IDF API / `idf.py` 명령 사용 | Arduino 프레임워크 사용 — ESP-IDF 직접 사용 금지 |
| DWM1000 SPI 핀(SS=5, RST=22)을 다른 용도로 재할당 | SPI 통신 불가 |
| `DW1000Ng::initialize()` (인터럽트 방식) | loop() 폴링 FSM과 일관성 유지 |
| `targetSpeed` 직접 대입 | `setTargetSpeed()` 미경유 시 FSM 게이팅 우회 |
| `analogWrite()` / `ledcWrite()` 모터 제어 | MCPWM API 사용 의무 |
| UWB 송신 후 `startReceive()` 미호출 | 반이중 UWB, 차기 수신 불가 |
| HTTP 핸들러에서 `controlMode` 검사 생략 | FSM 무력화로 제어 충돌 |
| 신호등 ESP32에서 삼변측량 계산 | RPi 역할 (3채널 통합 필요) |
| FSM 모드 전환을 HTTP 경로에서 트리거 | UWB 경로 전용으로 설계됨 |
