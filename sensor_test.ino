#include <Wire.h>

// ===== 시리얼/센서 설정 =====
const unsigned long SERIAL_BAUD = 115200;
const unsigned long SENSOR_INTERVAL_MS = 50;  // 20 Hz

// ===== 릴레이 핀 =====
const int L_R1 = 2;
const int L_R2 = 3;
const int R_R1 = 4;
const int R_R2 = 5;

// ===== PWM 핀 =====
const int L_PWM = 9;
const int R_PWM = 10;

// ===== 센서 핀 =====
const int PRESSURE_PIN = A0;  // 6847A
// GY-85: SDA=A4, SCL=A5
const byte ITG3205_ADDRESS = 0x68;
const byte ITG3205_WHO_AM_I = 0x00;
const byte ITG3205_SAMPLE_RATE_DIVIDER = 0x15;
const byte ITG3205_DLPF_FS = 0x16;
const byte ITG3205_GYRO_XOUT_H = 0x1D;
const byte ITG3205_POWER_MANAGEMENT = 0x3E;
const float ITG3205_SENSITIVITY = 14.375;  // LSB/(degree/second)

int speedValue = 150;
int leftTrim = 0;
int rightTrim = 0;
bool motorRunning = false;
bool gyroConnected = false;
unsigned long lastSensorTime = 0;

void writeI2CRegister(byte deviceAddress, byte registerAddress, byte value) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(value);
  Wire.endTransmission();
}

bool readI2CRegisters(
  byte deviceAddress,
  byte registerAddress,
  byte *buffer,
  byte length
) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  byte received = Wire.requestFrom(deviceAddress, length);
  if (received != length) {
    return false;
  }

  for (byte i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }

  return true;
}

bool setupGyro() {
  byte identity = 0;

  if (!readI2CRegisters(
        ITG3205_ADDRESS,
        ITG3205_WHO_AM_I,
        &identity,
        1
      )) {
    return false;
  }

  writeI2CRegister(ITG3205_ADDRESS, ITG3205_POWER_MANAGEMENT, 0x80);
  delay(10);
  writeI2CRegister(ITG3205_ADDRESS, ITG3205_SAMPLE_RATE_DIVIDER, 9);
  writeI2CRegister(ITG3205_ADDRESS, ITG3205_DLPF_FS, 0x1A);
  writeI2CRegister(ITG3205_ADDRESS, ITG3205_POWER_MANAGEMENT, 0x01);
  delay(10);

  return true;
}

bool readGyro(float &xDps, float &yDps, float &zDps) {
  byte data[6];

  if (!readI2CRegisters(
        ITG3205_ADDRESS,
        ITG3205_GYRO_XOUT_H,
        data,
        sizeof(data)
      )) {
    return false;
  }

  int16_t xRaw = (int16_t)((data[0] << 8) | data[1]);
  int16_t yRaw = (int16_t)((data[2] << 8) | data[3]);
  int16_t zRaw = (int16_t)((data[4] << 8) | data[5]);

  xDps = xRaw / ITG3205_SENSITIVITY;
  yDps = yRaw / ITG3205_SENSITIVITY;
  zDps = zRaw / ITG3205_SENSITIVITY;

  return true;
}

void publishSensors() {
  unsigned long now = millis();
  if (now - lastSensorTime < SENSOR_INTERVAL_MS) {
    return;
  }
  lastSensorTime = now;

  int pressureRaw = analogRead(PRESSURE_PIN);
  float gyroX = 0.0;
  float gyroY = 0.0;
  float gyroZ = 0.0;

  if (gyroConnected && !readGyro(gyroX, gyroY, gyroZ)) {
    gyroConnected = false;
  }

  // ROS2 브리지가 일반 로그와 구분할 수 있는 고정 CSV 형식:
  // SENSOR,<Arduino ms>,<pressure raw>,<gyro valid>,<x dps>,<y dps>,<z dps>
  Serial.print("SENSOR,");
  Serial.print(now);
  Serial.print(',');
  Serial.print(pressureRaw);
  Serial.print(',');
  Serial.print(gyroConnected ? 1 : 0);
  Serial.print(',');
  Serial.print(gyroX, 3);
  Serial.print(',');
  Serial.print(gyroY, 3);
  Serial.print(',');
  Serial.println(gyroZ, 3);
}

int leftSpeed() {
  return constrain(speedValue + leftTrim, 0, 255);
}

int rightSpeed() {
  return constrain(speedValue + rightTrim, 0, 255);
}

void printSpeed() {
  Serial.print("Base speed = ");
  Serial.print(speedValue);
  Serial.print(" | L = ");
  Serial.print(leftSpeed());
  Serial.print(" (trim ");
  Serial.print(leftTrim);
  Serial.print(") | R = ");
  Serial.print(rightSpeed());
  Serial.print(" (trim ");
  Serial.print(rightTrim);
  Serial.print(") | running = ");
  Serial.println(motorRunning ? "YES" : "NO");
}

void applyMotorSpeed() {
  if (!motorRunning) return;

  analogWrite(L_PWM, leftSpeed());
  analogWrite(R_PWM, rightSpeed());
}

void runMotor() {
  motorRunning = true;
  applyMotorSpeed();
}

void relayOn(int pin) {
  digitalWrite(pin, LOW);   // 릴레이 ON
}

void relayOff(int pin) {
  digitalWrite(pin, HIGH);  // 릴레이 OFF
}

void leftCW() {
  relayOff(L_R1);
  relayOff(L_R2);
}

void leftCCW() {
  relayOn(L_R1);
  relayOn(L_R2);
}

void rightCW() {
  relayOff(R_R1);
  relayOff(R_R2);
}

void rightCCW() {
  relayOn(R_R1);
  relayOn(R_R2);
}

void stopMotor() {
  motorRunning = false;
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}

void forward() {
  stopMotor();
  delay(100);

  // 실제 구동 확인 결과: 기존 turnLeft(a) 조합이 전진
  leftCCW();
  rightCW();

  runMotor();

  Serial.println("FORWARD");
}

void backward() {
  stopMotor();
  delay(100);

  // 실제 구동 확인 결과: 기존 turnRight(d) 조합이 후진
  leftCW();
  rightCCW();

  runMotor();

  Serial.println("BACKWARD");
}

void turnLeft() {
  stopMotor();
  delay(100);

  // 왼쪽 바퀴 후진 + 오른쪽 바퀴 전진
  leftCW();
  rightCW();

  runMotor();

  Serial.println("TURN LEFT");
}

void turnRight() {
  stopMotor();
  delay(100);

  // 왼쪽 바퀴 전진 + 오른쪽 바퀴 후진
  leftCCW();
  rightCCW();

  runMotor();

  Serial.println("TURN RIGHT");
}

void printSensors() {
  int pressureRaw = analogRead(PRESSURE_PIN);
  float gyroX = 0.0;
  float gyroY = 0.0;
  float gyroZ = 0.0;

  Serial.print("6847A A0 raw: ");
  Serial.print(pressureRaw);

  if (gyroConnected && readGyro(gyroX, gyroY, gyroZ)) {
    Serial.print(" | gyro dps: ");
    Serial.print(gyroX, 3);
    Serial.print(", ");
    Serial.print(gyroY, 3);
    Serial.print(", ");
    Serial.println(gyroZ, 3);
  } else {
    Serial.println(" | ITG-3205 gyro not detected");
  }
}

void setup() {
  pinMode(L_R1, OUTPUT);
  pinMode(L_R2, OUTPUT);
  pinMode(R_R1, OUTPUT);
  pinMode(R_R2, OUTPUT);

  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(PRESSURE_PIN, INPUT);

  relayOff(L_R1);
  relayOff(L_R2);
  relayOff(R_R1);
  relayOff(R_R2);
  stopMotor();

  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  gyroConnected = setupGyro();

  Serial.println("=== Robot Test ===");
  Serial.print("ITG-3205 gyro: ");
  Serial.println(gyroConnected ? "CONNECTED" : "NOT FOUND");
  Serial.println("w = forward");
  Serial.println("s = backward");
  Serial.println("a = turn left");
  Serial.println("d = turn right");
  Serial.println("x = stop");
  Serial.println("1~9 = base speed");
  Serial.println("[ / ] = left motor slower / faster");
  Serial.println(", / . = right motor slower / faster");
  Serial.println("0 = reset left/right trim");
  Serial.println("Speed changes apply immediately while moving");
  Serial.println("p = print sensors");
}

void loop() {
  publishSensors();

  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == '\n' || cmd == '\r') return;

    switch (cmd) {
      case 'w': forward(); break;
      case 's': backward(); break;
      case 'a': turnLeft(); break;
      case 'd': turnRight(); break;
      case 'x': stopMotor(); Serial.println("STOP"); break;
      case 'p': printSensors(); break;

      case '1': case '2': case '3':
      case '4': case '5': case '6':
      case '7': case '8': case '9':
        speedValue = map(cmd - '0', 1, 9, 30, 255);
        applyMotorSpeed();
        printSpeed();
        break;

      case '[':
        leftTrim -= 5;
        applyMotorSpeed();
        printSpeed();
        break;

      case ']':
        leftTrim += 5;
        applyMotorSpeed();
        printSpeed();
        break;

      case ',':
        rightTrim -= 5;
        applyMotorSpeed();
        printSpeed();
        break;

      case '.':
        rightTrim += 5;
        applyMotorSpeed();
        printSpeed();
        break;

      case '0':
        leftTrim = 0;
        rightTrim = 0;
        applyMotorSpeed();
        printSpeed();
        break;
    }
  }
}
