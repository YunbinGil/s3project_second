// ============================================================
//  esp32cam_sender.ino
//  역할: 카메라 캡처 → RPi로 HTTP POST 전송
//
//  ── 업로드 전 수정 ───────────────────────────
//  WIFI_SSID, WIFI_PASSWORD 를 본인 WiFi로 수정
// ============================================================

#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>

// ── WiFi 설정 ─────────────────────────────────
#define WIFI_SSID      "wifi 이름"
#define WIFI_PASSWORD  "wifi 비번"

// ── RPi 서버 주소 ─────────────────────────────
#define RPI_IP    "s3.local" //호스트네임.local
#define RPI_PORT  5000
#define RPI_URL   "http://" RPI_IP ":" STR(RPI_PORT) "/image"
#define STR(x)    STR2(x)
#define STR2(x)   #x

// ── 캡처 주기 (ms) ───────────────────────────
#define CAPTURE_INTERVAL_MS  500

// ── AI-Thinker ESP32-CAM 핀 고정 ─────────────
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

static unsigned long lastCapture = 0;

void initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    // PSRAM 있으면 고화질, 없으면 저화질
    if (psramFound()) {
        config.frame_size   = FRAMESIZE_VGA;   // 640x480
        config.jpeg_quality = 12;
        config.fb_count     = 2;
    } else {
        config.frame_size   = FRAMESIZE_QVGA;  // 320x240
        config.jpeg_quality = 20;
        config.fb_count     = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] 초기화 실패: 0x%x\n", err);
        return;
    }
    Serial.println("[CAM] 초기화 완료");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ESP32-CAM boot ===");

    initCamera();

    // WiFi 연결
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WiFi] 연결 중");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\n[WiFi] 연결됨: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WiFi] RPi 서버: http://%s:%d/image\n", RPI_IP, RPI_PORT);
}

void loop() {
    if (millis() - lastCapture < CAPTURE_INTERVAL_MS) return;
    lastCapture = millis();

    // 카메라 캡처
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[CAM] 캡처 실패");
        return;
    }

    // HTTP POST
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        char url[64];
        snprintf(url, sizeof(url), "http://%s:%d/image", RPI_IP, RPI_PORT);

        http.begin(url);
        http.addHeader("Content-Type", "image/jpeg");
        http.setTimeout(3000);

        int httpCode = http.POST(fb->buf, fb->len);

        if (httpCode == 200) {
            Serial.printf("[HTTP] 전송 성공: %d bytes\n", fb->len);
        } else {
            Serial.printf("[HTTP] 전송 실패: %d\n", httpCode);
        }
        http.end();
    } else {
        Serial.println("[WiFi] 연결 끊김, 재연결 시도");
        WiFi.reconnect();
    }

    esp_camera_fb_return(fb);
}
