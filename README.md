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
  --window-width-cm 120 \
  --window-height-cm 80 \
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
  gyro_z_bias_dps:=0.0
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
`gyro_z_dps`를 측정해 `gyro_z_bias_dps`로 설정해야 합니다.

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
  --window-width-cm 120 \
  --window-height-cm 80 \
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
  -p window_width_cm:=120.0 \
  -p window_height_cm:=80.0
```

Keep `auto_detect_window_corners:=false` when you want to use a hand-measured calibration file exactly as-is.


아두이노 모터 명령 계약:

```text
/arduino/motor_command geometry_msgs/Pose2D
x     이동할 x 거리 cm, dx_cm
y     이동할 y 거리 cm, dy_cm
theta 청소 플래그, 1.0이면 이동 후 청소
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
