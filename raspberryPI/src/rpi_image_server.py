#!/usr/bin/env python3
# ============================================================
#  rpi_image_server.py
#  역할: ESP32-CAM에서 보내는 JPEG 이미지 수신
#        수신된 이미지를 /tmp/latest.jpg 에 저장
#        (phase 7에서 OpenCV가 이 파일을 읽어 처리)
#
#  실행:
#    pip3 install flask
#    python3 rpi_image_server.py
# ============================================================

from flask import Flask, request
import time
import os

app = Flask(__name__)

SAVE_PATH = "/tmp/latest.jpg"
frame_count = 0
last_time = time.time()

@app.route('/image', methods=['POST'])
def receive_image():
    global frame_count, last_time

    data = request.data
    if not data:
        return "no data", 400

    # 파일 저장
    with open(SAVE_PATH, 'wb') as f:
        f.write(data)

    frame_count += 1
    now = time.time()
    fps = 1.0 / (now - last_time) if (now - last_time) > 0 else 0
    last_time = now

    print(f"[RX] frame={frame_count}  size={len(data)}bytes  fps={fps:.1f}")
    return "OK", 200

@app.route('/status', methods=['GET'])
def status():
    exists = os.path.exists(SAVE_PATH)
    return f"frames={frame_count} latest={SAVE_PATH} exists={exists}", 200

if __name__ == '__main__':
    print("=== RPi 이미지 서버 시작 ===")
    print(f"수신 주소: http://0.0.0.0:5000/image")
    print(f"저장 경로: {SAVE_PATH}")
    app.run(host='0.0.0.0', port=5000, debug=False)
