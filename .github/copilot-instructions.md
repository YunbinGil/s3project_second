# GitHub Copilot 전역 지시사항

이 워크스페이스의 모든 작업은 **반드시 프로젝트 루트의 `CLAUDE.md`를 최우선 참조 문서로 따라야 한다.**

## 필수 준수 사항

- 코드 생성, 수정, 리팩토링, 디버깅 등 **모든 작업 전**에 `CLAUDE.md`의 규칙을 확인할 것.
- `CLAUDE.md`에 명시된 **절대 금지 사항**은 어떠한 경우에도 위반하지 말 것.
- 특정 도메인 작업 시 `CLAUDE.md` 섹션 6의 **심화 규칙 파일 포인터**를 반드시 먼저 읽을 것.
- **계획 후 실행 원칙**: 코드를 즉시 생성하지 말고, 아키텍처 계획을 먼저 출력한 뒤 사용자 승인 후 구현할 것.

## 이중 빌드 환경 (절대 혼용 금지)

| 작업 도메인 | 사용 터미널 | 명령 체계 |
|------------|------------|---------|
| `raspberryPI/` 빌드·실행·디버그 | **VS Code Remote SSH 터미널** | `cmake`, `make`, Linux 시스템 명령 |
| `ESP32/` 빌드·플래시·모니터 | **Arduino IDE (로컬 PC)** | 스케치 컴파일·업로드, 시리얼 모니터 |

- `raspberryPI/` 작업 시 → **Remote SSH 터미널 기준**으로 경로·명령어 제시
- `ESP32/` 작업 시 → **Arduino IDE 기준**으로 `.ino` 스케치 및 `.h/.cpp` 파일 작성
- **RPi 터미널에서 `idf.py` 실행 제안 절대 금지**

## 환경 컨텍스트 (환각 방지)

- 호스트: **Raspberry Pi 4/5, Linux, C++17** (x86 리눅스나 Windows 절대 가정 금지)
- 주변기기: **신호등 ESP32 × 3 + 차량 ESP32 × 1, Arduino 프레임워크** (ESP-IDF 직접 사용 금지, `idf.py` 명령 제안 금지)
- 연결: **Remote SSH → Raspberry Pi**
- 신호등 ESP32: **DWM1000 UWB + UART2 (RXD2=GPIO16, TXD2=GPIO17)** — RPi와 UART 통신 + 차량과 UWB 거리 측정
- 차량 ESP32: **DWM1000 UWB + TB6612FNG 모터 (MCPWM) + WiFi AP** — RPi 미연결, 독립 동작

## 주요 금지 규칙 요약

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
| DWM1000 SPI 핀(SS=GPIO5, RST=GPIO22)을 다른 용도로 재할당 | SPI 통신 불가 |
| `DW1000Ng::initialize()` (인터럽트 방식) | loop() 폴링 FSM과 일관성 유지 |
| `targetSpeed` 직접 대입 | `setTargetSpeed()` 미경유 시 FSM 게이팅 우회 |
| `analogWrite()` / `ledcWrite()` 모터 제어 | MCPWM API 사용 의무 |
| UWB 송신 후 `startReceive()` 미호출 | 반이중 UWB, 차기 수신 불가 |
| HTTP 핸들러에서 `controlMode` 검사 생략 | FSM 무력화로 제어 충돌 |

> 상세 규칙은 `CLAUDE.md` 전문을 참조하라.
