# 🧪 ggul_bot

## 🥇 업데이트 정보

0. ggul_bot_v7_config
ggul_bot_v7_config 추가<br>
v6 -> v7 으로 변경

1. CMakeLists.txt <br>
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME}) 주석 처리 (assistant 파일을 프로그램이 못 찾음)<br>
- 우선 main 브런치에 이 부분은 계속 내가 업데이트 함 (필요시 주석 제거해도 됨)

2. ggul_bot_v6_config <br>
UDRF 참조 경로 수정 (상대경로에서 절대경로로)<br>

3. websocket_pkg, Strawberry_publisher_pkg 패키지 추가
<br><br>
## 🥈 websocket 제작 (라즈베리파이5 or 개인 노트북 → ros 터미널) <br>
- git clone 방법 동일, 이후 아래코드 실행해서 라이브러리 설치
<pre><code>git clone <꿀봇 저장소 URL></code></pre>
<pre><code>colcon build; source install/setup.bash</code></pre>
<pre><code>pip install websockets</code></pre>
- 패키지 사용
<pre><code>ros2 run websocket_pkg ws_to_ros_publisher</code></pre>
- vs code랑 연동하는 방법<br>
https://github.com/lko9911/RaspberryPi5-websocket<br><br>

## 🥈 Strawberry_publisher_pkg 제작 <br>
- git clone 방법 동일, 이후 아래코드 실행해서 라이브러리 설치
<pre><code>git clone https://github.com/lko9911/ggul_bot.git</code></pre>
<pre><code>colcon build; source install/setup.bash</code></pre>
<pre><code>cd /ros2_ws2/ggul_bot/strawberry_publisher_pkg/strawberry_publisher_pkg/
pip3 install -r requirements.txt </code></pre>
- pip 설치 코드
<pre><code>apt install python3-pip</code></pre>
- 패키지 사용
<pre><code>ros2 run strawberry_publisher_pkg strawberry_publisher</code></pre>
<br>

## 🥉 개선 사항
- [x] 코드 작동(Strawberry_Vision System) <br>
- [x] websocket 제작 (라즈베리파이5 or 개인 노트북 → ros 터미널)
- [x] VS code에서 main.py 로 ROS2 정보 넘기기
- [ ] IK 퍼블리쉬 (웹소켓으로 연결) <br>
- [ ] 55cm 전방 캘리브레이션 하드코딩
- [ ] ROS2 터미널에서 정보 받아오기 (값 변환은 vs code에서 관리)


