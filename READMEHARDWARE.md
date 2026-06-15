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

| 키   | 기능    |
| --- | ----- |
| w   | 전진    |
| s   | 후진    |
| a   | 좌회전   |
| d   | 우회전   |
| x   | 정지    |
| 1~9 | 속도 설정 |
| p   | 센서 출력 |

---

# 속도 설정

```text
1 = 매우 느림
5 = 중간
9 = 최대
```

PWM 변환

```cpp
speedValue = map(cmd - '0', 1, 9, 30, 255);
```

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
