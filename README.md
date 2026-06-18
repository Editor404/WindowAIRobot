# Window Cleaner ROS2 Package

YOLOv8 segmentation model (`seg_best.pt`)로 오염 영역을 찾고, 마스크 중심 픽셀을 로봇 좌표계의 절대 좌표로 변환하는 ROS2 패키지입니다.

타깃 보드는 Raspberry Pi 5 + Ubuntu 24.04라서 ROS2 Jazzy 기준으로 사용합니다.

## 설치

```bash
python3 -m pip install -r requirements.txt
```

ROS2 워크스페이스 구조에서는 이 폴더를 `src/window_cleaner` 위치에 두고 빌드합니다.

```bash
cd ~/capstone_ws
colcon build --packages-select window_cleaner
source install/setup.bash
```

라즈베리파이5 Ubuntu 24.04 세팅은 [docs/raspberry_pi5_ubuntu24_jazzy.md](docs/raspberry_pi5_ubuntu24_jazzy.md)를 참고합니다.

## 주요 노드

### dirt_target_node

`/camera/image_raw`와 `/robot_pose`를 구독하고, 오염의 절대 좌표를 `/robot/target_pose`로 발행합니다.

```text
subscribe:
  /camera/image_raw  sensor_msgs/Image
  /robot_pose        geometry_msgs/Pose2D

publish:
  /robot/target_pose geometry_msgs/Pose2D
```

실행:

```bash
ros2 run window_cleaner dirt_target_node \
  --ros-args \
  -p image_topic:=/camera/image_raw \
  -p image_width:=0 \
  -p image_height:=0 \
  -p cm_per_pixel_x:=0.05 \
  -p cm_per_pixel_y:=0.05 \
  -p camera_offset_x_cm:=0.0 \
  -p camera_offset_y_cm:=0.0 \
  -p device:=cpu
```


영상 입력 파라미터:

```text
image_topic   입력 영상 topic 이름
image_width   노드 내부 처리 width, 0이면 원본 크기 유지
image_height  노드 내부 처리 height, 0이면 원본 크기 유지
```

오염인식과 localization은 저장된 이미지가 아니라 `image_topic`으로 들어오는 영상을 사용합니다.

### dirt_segmentation_node

카메라 이미지만 받아서 오염 중심 픽셀을 `/dirt/center_pixel`로 발행합니다.

```text
subscribe:
  /camera/image_raw    sensor_msgs/Image

publish:
  /dirt/center_pixel   geometry_msgs/PointStamped
```

### robot_controller_node

목표 좌표를 받아 아두이노 우노 구동단이 구독할 모터 명령 토픽을 발행합니다.

```text
subscribe:
  /robot/target_pose geometry_msgs/Pose2D

publish:
  /arduino/motor_command geometry_msgs/Pose2D
```


### Homography localization modes

`dirt_target_node` can publish dirt positions in the `window_frame` using `config/window_calibration.yaml`.

```text
window_width_cm/window_height_cm 없음 -> normalized mode
  publish: /dirt/window_position_normalized geometry_msgs/PointStamped
  point.x, point.y = 0.0 ~ 1.0

window_width_cm/window_height_cm 있음 -> metric mode
  publish: /dirt/window_position_cm geometry_msgs/PointStamped
  point.x, point.y = cm
```

Corner order in calibration is always:

```text
top_left, top_right, bottom_right, bottom_left
```

`PointStamped.point.z` stores the dirt detection confidence.

### Automatic window corner detection

기본 실행 경로는 카메라 영상을 `calib_parameters.npz`로 왜곡 보정한 뒤,
`best.pt`의 `window_frame`/`window_glass` segmentation 결과로 창문 코너를 검출합니다.
AI 검출이 실패하면 기존 OpenCV rule-based 검출로 자동 fallback합니다.

```text
/camera/image_raw
  -> camera undistortion (calib_parameters.npz)
  -> best.pt segmentation
  -> frame/glass boundary mask
  -> TL, TR, BR, BL corner extraction
  -> homography
  -> dirt position in window_frame
```

주요 파라미터:

```text
auto_detect_window_corners              true
window_corner_detector                  ai | rule_based
window_model_path                       best.pt
window_confidence_threshold             0.5
window_detector_fallback_to_rule_based  true
undistort_image                         true
camera_calibration_path                 calib_parameters.npz
camera_calibration_width/height         640 / 480
```

`CornerDetection.py`는 동일한 AI 검출 파이프라인을 단일 이미지에서 확인하는 도구입니다.

```bash
python3 CornerDetection.py \
  --image sample_window.jpg \
  --window-width-cm 80 \
  --window-height-cm 160 \
  --map-bottom-left-x-cm 0 \
  --map-bottom-left-y-cm 0 \
  --camera-map-x-cm 15 \
  --camera-map-y-cm 30 \
  --bim-output window_bim.json \
  --output detected_window.jpg
```

로봇의 최초 위치를 맵의 좌하단 원점으로 두고, 검출된 좌상단·우상단을
2D BIM의 상단 경계 관측값으로 기록합니다. 직사각형 BIM은 입력한 실제 폭·높이로
`BL -> BR -> TR -> TL` 네 점을 생성합니다. PnP는 수학적으로 대응점 두 개만으로
계산할 수 없으므로 동일 검출 결과의 네 영상 코너를 사용하며, 결과 JSON에는
회전 벡터, 이동 벡터(cm), 회전 행렬, 창문 좌표계에서의 카메라 위치와 재투영 오차가
저장됩니다. 폭·높이를 생략하면 기존처럼 코너 검출 이미지만 생성합니다.
기본 맵 기준은 좌하단 `(0, 0) cm`, 초기 카메라 위치 `(15, 30) cm`이며 JSON의
`source_observations.camera_map_position_cm`에 기록됩니다.

### Arduino GY-85 / ITG-3205

GY-85 보드의 ITG-3205 자이로를 Arduino에서 20 Hz로 읽어 USB 직렬로 전달합니다.
스케치는 `arduino/gy85_itg3205_telemetry/gy85_itg3205_telemetry.ino`에 있습니다.

```text
SENSOR,<millis>,<pressure_raw>,<gyro_valid>,<x_dps>,<y_dps>,<z_dps>,<blower_pwm>,<adhesion_secure>
```

ROS2 브리지 실행:

```bash
ros2 launch window_cleaner arduino_imu.launch.py \
  port:=/dev/ttyACM0 \
  gyro_z_bias_dps:=-1.35
```

발행 토픽:

```text
/gyro/data      sensor_msgs/Imu    각속도와 적분된 yaw
/robot/imu_pose geometry_msgs/Pose2D
  x=15 cm, y=30 cm, theta=yaw(rad)
/pressure/raw   std_msgs/Int32
/adhesion/secure std_msgs/Bool
```

`robot_controller_node`는 `/robot/imu_pose.theta`를 사용하여 맵 좌표계의 목표 이동량을
로봇 기준 이동량으로 변환한 뒤 `/arduino/motor_command`로 보냅니다. IMU 데이터가
아직 수신되지 않았으면 기존 맵 축 기준 이동량을 사용합니다.

ITG-3205 각속도는 시간 적분 시 드리프트가 발생하므로 정지 상태의 평균
`gyro_z_dps`를 측정해 `gyro_z_bias_dps`로 설정해야 합니다. 현재 측정된 정지 상태 평균은 약 `-1.35 dps`입니다.

### 압력 기반 흡착 블로워 제어

블로워 드라이버의 `VSR` 입력은 Arduino D11의 약 31 kHz PWM으로 제어합니다.
`EN`은 D7, `FR`은 D8에 연결하며 Arduino GND와 24 V 드라이버의 신호 GND를
공통으로 연결합니다. 24 V 모터 전원은 Arduino에서 공급하면 안 됩니다.
이 핀 추가는 기존 `sensor_test.ino`의 D2–D5 릴레이, D9/D10 주행 PWM,
A0 압력센서, A4/A5 GY-85 배선을 그대로 보존합니다.

압력센서 A0 값에는 히스테리시스를 적용합니다.

```text
ADHESION_TARGET_RAW  이상 → 흡착 확보
ADHESION_RELEASE_RAW 미만 → 흡착 손실
흡착 손실/센서 단선·단락 → 블로워 최대 출력 + 주행 명령 차단
```

`ADHESION_TARGET_RAW`, `ADHESION_RELEASE_RAW` 및
`PRESSURE_INCREASES_WITH_SUCTION`은 실제 대기압/흡착 상태 ADC 측정값에 맞춰
반드시 보정해야 합니다.

기존 `calibrate_window` 명령은 모델 없이 OpenCV만으로 사각형을 검출하고
`config/window_calibration.yaml`을 작성하는 수동 fallback 도구로 유지됩니다.

```bash
ros2 run window_cleaner calibrate_window \
  --image sample_window.jpg \
  --window-width-cm 80 \
  --window-height-cm 160 \
  --output config/window_calibration.yaml \
  --annotated-output detected_window.jpg
```

If `--image` is omitted, one frame is captured from `--camera` instead. The detector uses Canny edges, contour extraction, polygon approximation, then selects the largest convex quadrilateral. Detected corners are saved in this order:

```text
top_left, top_right, bottom_right, bottom_left
```

`dirt_target_node` can also auto-detect corners on the first camera frame when enabled:

```bash
ros2 run window_cleaner dirt_target_node \
  --ros-args \
  -p auto_detect_window_corners:=true \
  -p auto_save_window_calibration:=true \
  -p window_width_cm:=80.0 \
  -p window_height_cm:=160.0
```

Keep `auto_detect_window_corners:=false` when you want to use a hand-measured calibration file exactly as-is.


아두이노 모터 명령 계약:

```text
/arduino/motor_command geometry_msgs/Pose2D
x     이동할 x 거리 cm, dx_cm
y     이동할 y 거리 cm, dy_cm
theta 청소 플래그, 1.0이면 이동 후 청소
```


### SSH 환경 영상 확인

GUI 없이 라즈베리파이에 SSH로 접속한 경우 `mjpeg_stream`으로 카메라 토픽을 웹 브라우저에서 볼 수 있습니다.

```bash
# 터미널 1: 카메라 publish
ros2 run window_cleaner rpi_camera_node --ros-args -p backend:=rpicam

# 터미널 2: MJPEG 웹 스트림
source install/setup.bash
ros2 run window_cleaner mjpeg_stream --ros-args \
  -p image_topic:=/camera/image_raw \
  -p port:=8080
```

같은 네트워크의 노트북/PC 브라우저에서 아래 주소를 엽니다.

```text
http://<라즈베리파이IP>:8080
```

스냅샷만 저장하려면:

```bash
ros2 run window_cleaner image_snapshot --ros-args \
  -p image_topic:=/camera/image_raw \
  -p output:=/tmp/window_cleaner_snapshot.jpg
```

## Launch 실행

```bash
ros2 launch window_cleaner dirt_target.launch.py
```

## 캘리브레이션

`cm_per_pixel_x`, `cm_per_pixel_y`는 카메라 캘리브레이션으로 구해야 합니다. 예를 들어 화면에서 200px 길이가 실제 10cm라면 `10 / 200 = 0.05`입니다.

## 좌표 계산식

```text
오염 절대 x = 로봇 현재 x + 카메라 x 오프셋 + (오염 중심 px - 화면 중심 px) * x축 cm/px
오염 절대 y = 로봇 현재 y + 카메라 y 오프셋 + (화면 중심 py - 오염 중심 py) * y축 cm/px
```

## Gazebo Classic 시뮬레이션

이 패키지는 실제 하드웨어 없이 Gazebo Classic에서 최소 통합 테스트를 할 수 있는
데모 world와 시뮬레이션 브리지를 포함합니다.

구성:

```text
worlds/window_cleaner_demo.world  실측 유리 80×160cm + 5cm 폭·+X 20cm 돌출 창틀/장착 카메라/로봇 모델 world
urdf/window_cleaner_robot.urdf    jagosipda.stl 기반 30×30cm 로봇 visual 모델
meshes/jagosipda.stl              Gazebo에서 표시할 로봇 STL mesh
launch/gazebo_sim.launch.py       Gazebo world + 비전 노드 + 컨트롤러 + sim bridge 실행
sim_bridge_node                   /arduino/motor_command를 받아 가상 pose/흡착 토픽 발행
```

월드 구성:

- `traction_test_floor`: 팬/흡착 없이 주행 마찰을 먼저 확인하는 수평 바닥
- `window_wall`: 유리 좌하단 (0,0,0), 유리 80×160cm, 유리 주변 5cm 창틀, +X 방향 20cm 돌출
- `white_dirt_dot_*`: 유리 -X쪽 표면의 랜덤 위치 하얀색 오염 점
- `robot_start_marker`: Gazebo pose 기준점을 보여주는 시작 위치 마커
- `robot_camera`: 로봇에 장착되어 `/robot_camera/image_raw`, `/robot_camera/camera_info`를 발행하는 Gazebo ROS 카메라
- `window_cleaner_robot`: world에 직접 포함된 `jagosipda.stl` visual mesh 로봇

ROS2 Humble 기준 설치:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs python3-colcon-common-extensions
```

빌드:

```bash
mkdir -p ~/capstone_ws/src
ln -s /home/keivn/capstone ~/capstone_ws/src/window_cleaner
cd ~/capstone_ws
source /opt/ros/humble/setup.bash
python3 -m pip install -r src/window_cleaner/requirements.txt
colcon build --packages-select window_cleaner
source install/setup.bash
```

실행:

```bash
# 여러 터미널/rqt에서 토픽을 볼 경우 모든 터미널의 ROS_LOCALHOST_ONLY 값을 동일하게 맞춥니다.
# 기본값을 쓸 때는 unset 상태로 둡니다. localhost-only로 쓸 때는 실행/확인 터미널 모두 export 하세요.
# export ROS_LOCALHOST_ONLY=1
ros2 launch window_cleaner gazebo_sim.launch.py
```

확인:

```bash
# ROS_LOCALHOST_ONLY=1로 실행했다면 이 확인 터미널에도 같은 값을 export 해야 합니다.
ros2 daemon stop
ros2 node list --no-daemon
ros2 topic list -v --no-daemon
ros2 topic hz /robot_camera/image_raw
ros2 topic echo /robot/target_pose
ros2 topic echo /arduino/motor_command
ros2 topic echo /robot_pose
```

Gazebo 런치는 YOLO 대신 시뮬레이션용 흰색/회색 오염점 검출기(`detector_mode:=sim`)를 사용합니다. 현재 제어 구조는 `/robot_camera/image_raw` → `/dirt/detections_px` → `window_cleaning_planner_node` → `/robot/target_pose` → `/arduino/motor_command` → Gazebo 모델 이동 순서입니다. 플래너는 80×160cm 직사각형 유리 맵에서 lawn-mower 스캔 waypoint를 돌면서 보이는 오염점을 `/dirt/memory_count`에 누적하고, 스캔 후 가까운 오염점부터 청소 target으로 발행합니다. 컨트롤러/브리지만 따로 확인하려면 아래처럼 목표 좌표를 수동 발행할 수 있습니다.

```bash
ros2 topic pub --once /robot/target_pose geometry_msgs/msg/Pose2D \
"{x: 30.0, y: 45.0, theta: 1.0}"
```

현재 Gazebo 구성은 `jagosipda.stl` 로봇 외형을 Gazebo에 spawn하지만, 아직 트랙 구동/흡착
force 플러그인까지 물리 구현한 단계는 아닙니다. 실제 제어 파이프라인은 `sim_bridge_node`가
`/arduino/motor_command`를 단순 pose 변화로 적분해서 검증합니다.

Gazebo 창틀은 유리 주변 5 cm 폭에 +X 방향 20 cm 돌출로 설정했습니다. 로봇은 창문
수직 유리면의 +X쪽 반대편에 바닥면이 닿도록 배치됩니다. heading이 바뀌어도 로봇 local -Z underside가 world -X 유리 방향을 유지하도록 자세를 계산하며, 로봇 top-view 좌하단이 유리 좌하단 원점 `(0,0,0)`에 오도록 초기 중심은 `(15,15) cm`이고 heading 0은 유리 위쪽(world +Z)을 향합니다.
유리 두께 2 cm의 +X쪽 표면은 `X=+1 cm`이고, `jagosipda.stl`의 실제 visual 바닥면
기준으로 로봇 중심을 `X=+7.8886 cm`에 고정해서 로봇 바닥면이 창틀이 아니라 중앙
유리 표면에 닿도록 유지합니다. Gazebo collision이 창틀/유리와 충돌해 밀어내지 않도록
로봇 physics collision은 제거하고, underside visual pad로 접촉면을 표시합니다.
`sim_bridge_node` 이동도 창문 좌하단 기준
`(x_cm, y_cm) -> Gazebo world (Y, Z)`로 반영합니다.

로봇 장착 카메라는 30 x 30 cm 로봇 top-view의 좌하단을 `(0, 0)`으로 봤을 때
`(15, 30) cm` 위치에 있으며, 로봇 바닥면 기준 높이는 `13 cm`입니다. 카메라는 X축 기준 회전 roll을 `0도`로 고정하고, `(15,30) cm` 위치에서 유리 상단 좌/우 코너 `(0,160)`, `(80,160)`이 모두 들어오도록 두 코너의 각도 중간 방향에 맞췄습니다. 현재 로컬 yaw는 약 `-10.0도`, pitch는 약 `5.6도`입니다. 실제 카메라 스펙에 맞춰 horizontal AOV는 `54도`, 640×480 기준 vertical AOV는 약 `42도`, update rate는 `30fps`로 설정했습니다. 이 카메라가 `/robot_camera/image_raw`와 `/robot_camera/camera_info`를 발행합니다.
고정 관찰용 월드 카메라는 토픽 충돌을 피하기 위해 `/world_camera/image_raw`를 발행합니다.

Gazebo 흰점 삭제:

- 오염점은 `sim_dirt_dot_XX` 개별 모델로 생성됩니다.
- `gazebo_dirt_cleaner_node`가 `/robot_pose`를 보고 로봇 30×30cm footprint 안에 들어온 오염 모델을 Gazebo delete service로 삭제합니다.
- 남은 시뮬레이션 오염점 수는 `/dirt/sim_remaining_count`, 삭제된 오염점 이름은 `/dirt/sim_cleaned`에서 확인합니다.
