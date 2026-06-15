# Raspberry Pi 5 Ubuntu 24.04 ROS2 Jazzy Setup

이 프로젝트는 Raspberry Pi 5 + Ubuntu 24.04 64-bit + ROS2 Jazzy에서 실행하는 것을 기준으로 합니다.

## 1. ROS2 Jazzy 설치

Ubuntu 24.04에서는 ROS2 Jazzy를 사용합니다. Humble은 Ubuntu 22.04 기준이라 24.04 타깃에서는 Jazzy로 가는 편이 맞습니다.

ROS2 설치 후 아래 명령이 동작해야 합니다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

필요한 ROS 패키지는 다음과 같습니다.

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-rclpy \
  ros-jazzy-cv-bridge \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-launch \
  ros-jazzy-launch-ros
```

## 2. Python 의존성 설치

YOLOv8 추론에는 `ultralytics`, `torch`, `opencv`, `numpy`가 필요합니다.

```bash
cd ~/capstone_ws/src/window_cleaner
python3 -m pip install -r requirements.txt
```

라즈베리파이에서 pip가 시스템 패키지 설치를 막으면 가상환경을 사용합니다.

```bash
cd ~/capstone_ws
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r src/window_cleaner/requirements.txt
```

`--system-site-packages`를 쓰는 이유는 ROS2의 `rclpy`, `cv_bridge` 같은 apt 설치 Python 패키지를 가상환경에서도 보이게 하기 위해서입니다.

## 3. 워크스페이스 배치

현재 폴더를 ROS2 워크스페이스의 `src/window_cleaner`로 둡니다.

```text
capstone_ws/
  src/
    window_cleaner/
      package.xml
      setup.py
      seg_best.pt
      window_cleaner/
```

이미 다른 위치에 있다면 예시는 다음과 같습니다.

```bash
mkdir -p ~/capstone_ws/src
cp -r ~/capstone ~/capstone_ws/src/window_cleaner
```

## 4. 빌드

```bash
cd ~/capstone_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select window_cleaner
source install/setup.bash
```

빌드 후 노드가 보이는지 확인합니다.

```bash
ros2 pkg executables window_cleaner
```

## 5. 실행

카메라 토픽과 로봇 현재 좌표 토픽이 이미 들어온다는 가정입니다.

```bash
ros2 launch window_cleaner dirt_target.launch.py
```

직접 파라미터를 넣어서 실행할 수도 있습니다.

```bash
ros2 run window_cleaner dirt_target_node \
  --ros-args \
  -p image_topic:=/camera/image_raw \
  -p robot_pose_topic:=/robot_pose \
  -p target_topic:=/robot/target_pose \
  -p cm_per_pixel_x:=0.05 \
  -p cm_per_pixel_y:=0.05 \
  -p camera_offset_x_cm:=0.0 \
  -p camera_offset_y_cm:=0.0 \
  -p device:=cpu
```

## 6. 토픽 확인

```bash
ros2 topic list
ros2 topic echo /robot/target_pose
ros2 topic hz /camera/image_raw
```

로봇 현재 좌표를 임시로 넣어 테스트하려면:

```bash
ros2 topic pub /robot_pose geometry_msgs/msg/Pose2D "{x: 0.0, y: 0.0, theta: 0.0}"
```

## 7. 성능 메모

Raspberry Pi 5에서는 `seg_best.pt`를 CPU로 실행하면 프레임레이트가 낮을 수 있습니다. 먼저 `device:=cpu`로 안정성을 확인하고, 이후 필요하면 모델 경량화나 TensorRT/NCNN 같은 변환을 검토합니다.

캡스톤 시연에서는 처음부터 고프레임 실시간 처리보다 다음 순서가 더 안전합니다.

```text
1. 카메라 프레임 1장 입력
2. 오염 검출
3. 목표 좌표 발행
4. 로봇 이동
5. 다음 프레임 처리
```

## 8. 현재 코드의 하드웨어 연결 위치

실제 모터 제어는 아직 자리표시자입니다.

수정할 파일:

```text
window_cleaner/robot_controller_node.py
```

현재는 `/robot/target_pose`를 받으면 `/arduino/motor_command`로 `geometry_msgs/Pose2D` 명령을 발행합니다. 아두이노 우노 쪽 브리지/스케치가 이 토픽을 구독해서 모터를 구동해야 합니다.
