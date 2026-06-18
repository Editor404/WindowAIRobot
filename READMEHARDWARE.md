# Window Cleaning Robot Motor Control System

## 시스템 구성

### 하드웨어

* Arduino Uno
* 12V 모터 컨트롤러 × 2
* 12V DC 모터 × 2
* 5V 4채널 릴레이 모듈 × 1
* MPX/6847A 압력센서
* GY-85 IMU 센서

---

# 핀맵

## 릴레이 모듈

| Arduino | Relay |
| ------- | ----- |
| D2      | IN1   |
| D3      | IN2   |
| D4      | IN3   |
| D5      | IN4   |
| 5V      | VCC   |
| GND     | GND   |

릴레이 모듈은 Active LOW 타입

```cpp
LOW  = Relay ON
HIGH = Relay OFF
```

---

## PWM 출력

| Arduino | 기능        |
| ------- | --------- |
| D9      | Left PWM  |
| D10     | Right PWM |

PWM 범위

```cpp
0 ~ 255
```

---

## 압력센서

6847A

| Sensor | Arduino |
| ------ | ------- |
| VCC    | 5V      |
| GND    | GND     |
| OUT    | A0      |

단독 확인 스케치는 `arduino/pressure_6847a_monitor/pressure_6847a_monitor.ino`에 있다.
Arduino IDE에서 업로드한 뒤 시리얼 모니터를 `115200 baud`로 열면 기본 1초마다 아래 형식으로 출력된다.

```text
PRESSURE,<millis>,<raw>,<voltage_v>
```

더 빠르게 보고 싶으면 스케치의 `SAMPLE_INTERVAL_MS`를 줄이고, 변화가 있을 때만 보고 싶으면
`PRINT_ONLY_IF_RAW_CHANGED_BY`를 예를 들어 `5`나 `10`으로 바꾼다.

확인 순서:
1. 센서를 대기압 상태로 두고 raw 값을 기록한다.
2. 흡착 챔버에 연결하고 WS7040을 켠 상태의 raw 값을 기록한다.
3. 손으로 로봇을 유리 쪽으로 더 눌렀을 때 raw 값이 어느 방향으로 변하는지 본다.

이 세 값으로 `PRESSURE_INCREASES_WITH_SUCTION`, `ADHESION_TARGET_RAW`,
`ADHESION_RELEASE_RAW`를 정한다.

현재 실측 기준:

```text
WS7040 OFF:       raw ≈ 913
WS7040 full load: raw ≈ 871
```

따라서 흡착이 강할수록 raw 값이 내려가는 방향이다. 통합 스케치 기준 provisional 값은:

```cpp
PRESSURE_INCREASES_WITH_SUCTION = false;
ADHESION_TARGET_RAW = 885;   // 이 값 이하이면 흡착 확보
ADHESION_RELEASE_RAW = 900;  // 이 값 초과이면 흡착 손실
```

---

## GY-85

| GY-85 | Arduino |
| ----- | ------- |
| VCC   | 5V      |
| GND   | GND     |
| SDA   | A4      |
| SCL   | A5      |

## 흡착 블로워 드라이버 추가

기존 `sensor_test.ino` 배선은 유지하고 사용하지 않는 핀에 연결합니다.

| Blower driver | Arduino |
| --- | --- |
| VSR (PWM 2–50 kHz, 0–5 V) | D11, 약 31 kHz |
| EN | D7 |
| FR | D8 |
| Signal GND | GND |

기존 D2–D5 릴레이, D9/D10 주행 PWM, A0 압력센서, A4/A5 GY-85 연결은
변경하지 않습니다. 블로워 24 V 전원은 Arduino 5 V와 분리하고 신호 GND만
공통 연결합니다.

---

# 모터 컨트롤러 정역 입력 구조

컨트롤러 4핀 입력

```text
A = 기판1
B = 기판2
C = 기판3
D = 기판4
```

실제 배선 예

```text
A = 빨강
B = 노랑
C = 검정
D = 파랑
```

---

# 확인된 동작

CW

```text
A ↔ C
B ↔ D
```

CCW

```text
A ↔ D
B ↔ C
```

STOP

```text
PWM = 0
```

---

# 릴레이 배선

## 왼쪽 모터

### Relay 1

```text
COM -> A
NC  -> C
NO  -> D
```

### Relay 2

```text
COM -> B
NC  -> D
NO  -> C
```

---

## 오른쪽 모터

### Relay 3

```text
COM -> A
NC  -> C
NO  -> D
```

### Relay 4

```text
COM -> B
NC  -> D
NO  -> C
```

---

# 릴레이 상태

## CW

```cpp
Relay1 OFF
Relay2 OFF

Relay3 OFF
Relay4 OFF
```

실제 출력

```text
A-C
B-D
```

---

## CCW

```cpp
Relay1 ON
Relay2 ON

Relay3 ON
Relay4 ON
```

실제 출력

```text
A-D
B-C
```

---

# 시리얼 명령

Arduino IDE 시리얼 모니터에서 보드레이트는 `115200`, 줄 끝은 `Newline` 또는
`Both NL & CR`로 둔다. 기본 직진 보정 PWM은 왼쪽 `150`, 오른쪽 `235`이다.

| 입력 | 기능 |
| --- | --- |
| `L150` | 왼쪽 궤도 PWM을 150으로 설정 |
| `R235` | 오른쪽 궤도 PWM을 235로 설정 |
| `P150,235` | 왼쪽/오른쪽 PWM을 동시에 설정 |
| `w` | 전진 |
| `s` | 후진 |
| `a` | 좌회전 |
| `d` | 우회전 |
| `x` | 정지 |
| `?` | 현재 `left_pwm`, `right_pwm`, `blower_pwm`, `adhesion_secure` 출력 |

압력/흡착 상태는 20 Hz로 아래 형식으로 계속 출력된다.

```text
SENSOR,<millis>,<pressure_raw>,<gyro_valid>,<x_dps>,<y_dps>,<z_dps>,<blower_pwm>,<adhesion_secure>
```

흡착이 확보되지 않으면 주행 명령은 `DRIVE_BLOCKED,ADHESION_NOT_SECURE`로 차단된다.


`1`~`9`는 기존 호환용 단축 속도 명령이다. 이 명령은 좌우 PWM을 같은 값으로
바꾸므로, 직진 보정 실험 중에는 `P150,235`처럼 좌우 분리 명령을 우선 사용한다.

---

# 동작 정의

## 전진

```text
왼쪽 CW
오른쪽 CW
```

---

## 후진

```text
왼쪽 CCW
오른쪽 CCW
```

---

## 좌회전

```text
왼쪽 CCW
오른쪽 CW
```

---

## 우회전

```text
왼쪽 CW
오른쪽 CCW
```

---

# 안전사항

방향 전환 전 반드시 PWM을 0으로 설정

```cpp
analogWrite(L_PWM, 0);
analogWrite(R_PWM, 0);

delay(100);
```

권장

```cpp
CW -> STOP -> 100ms -> CCW
```

금지

```cpp
CW -> 즉시 CCW
```

---

# 센서 확인

압력센서

```cpp
analogRead(A0);
```

GY-85

```cpp
Wire.begin();
```

현재 SDA=A4, SCL=A5 연결 완료.

---

# 최종 시스템 구조

```text
PC
│
├─ Serial
│
▼
Arduino Uno
│
├─ Relay1/2 → Left Motor Controller
├─ Relay3/4 → Right Motor Controller
│
├─ PWM D9  → Left Controller
├─ PWM D10 → Right Controller
│
├─ A0      → 6847A Pressure Sensor
│
└─ I2C
    ├─ SDA A4
    └─ SCL A5
         ↓
       GY-85
```
