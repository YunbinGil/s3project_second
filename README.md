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

## 참고 문서

- [CLAUDE.md](CLAUDE.md) — 프로젝트 헌장 (AI 코딩 규칙 전문)
- [preCode/motor.ino](preCode/motor.ino) — 차량 ESP32 참조 코드 (이전 프로젝트)
- [preCode/mqtt_esp32.ino](preCode/mqtt_esp32.ino) — UART+UWB 통신 참조 코드
- [DW1000Ng 라이브러리](https://github.com/F-Army/DW1000Ng) — DWM1000 Arduino 라이브러리

---

## 라이선스

이 프로젝트는 학습/연구 목적으로 작성되었습니다.
