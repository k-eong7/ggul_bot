#!/bin/bash

echo "🔧 slcan 초기화 중..."
# 1. 기존 slcand 종료 (중복 방지)
sudo pkill slcand

# 2. slcan 커널 모듈 로드
sudo modprobe slcan

# 3. CANable을 slcan0으로 연결 (250 kbps = -s5)
sudo slcand -o -c -s5 /dev/ttyACM0 slcan0

# 4. 인터페이스 활성화
sudo ip link set up slcan0
echo "✅ slcan0 활성화 완료"

# 5. Python 스크립트 실행
echo "🚀 Python 스크립트 실행 중..."
