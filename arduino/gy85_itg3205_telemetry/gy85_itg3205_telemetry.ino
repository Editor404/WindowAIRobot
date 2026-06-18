#include <Wire.h>
#include <string.h>

const unsigned long SERIAL_BAUD = 115200;
const unsigned long SENSOR_INTERVAL_MS = 50;
const byte COMMAND_BUFFER_SIZE = 24;

// sensor_test.ino 기본 배선: 변경하지 않는다.
const int LEFT_RELAY_1_PIN = 2;
const int LEFT_RELAY_2_PIN = 3;
const int RIGHT_RELAY_1_PIN = 4;
const int RIGHT_RELAY_2_PIN = 5;
const int LEFT_DRIVE_PWM_PIN = 9;
const int RIGHT_DRIVE_PWM_PIN = 10;
const int PRESSURE_PIN = A0;

// 흡착 블로워 드라이버 추가 배선.
const int BLOWER_PWM_PIN = 11;  // Timer2 OC2A, approximately 31 kHz
const int BLOWER_ENABLE_PIN = 7;
const int BLOWER_DIRECTION_PIN = 8;

// 반드시 실제 센서 측정값으로 조정한다.
// 기본값은 흡착이 강할수록 ADC 값이 증가한다고 가정한다.
const bool PRESSURE_INCREASES_WITH_SUCTION = false;
const int ADHESION_TARGET_RAW = 885;
const int ADHESION_RELEASE_RAW = 900;
const int PRESSURE_SENSOR_MIN_RAW = 5;
const int PRESSURE_SENSOR_MAX_RAW = 1018;
const int BLOWER_MIN_PWM = 80;
const int BLOWER_MAX_PWM = 255;
const int BLOWER_START_PWM = 220;
const int CONTROL_STEP = 4;
const int CONTROL_RELAX_MARGIN_RAW = 10;
const unsigned long STARTUP_GRACE_MS = 3000;
const byte ITG3205_ADDRESS = 0x68;
const byte WHO_AM_I = 0x00;
const byte SAMPLE_RATE_DIVIDER = 0x15;
const byte DLPF_FS = 0x16;
const byte GYRO_XOUT_H = 0x1D;
const byte POWER_MANAGEMENT = 0x3E;
const float GYRO_SENSITIVITY = 14.375;

unsigned long lastSensorTime = 0;
bool gyroConnected = false;
bool adhesionSecure = false;
int blowerPwm = BLOWER_START_PWM;
unsigned long startupTime = 0;
int leftDriveSpeed = 150;
int rightDriveSpeed = 235;
bool driveRunning = false;
char commandBuffer[COMMAND_BUFFER_SIZE];
byte commandLength = 0;

void writeRegister(byte registerAddress, byte value) {
  Wire.beginTransmission(ITG3205_ADDRESS);
  Wire.write(registerAddress);
  Wire.write(value);
  Wire.endTransmission();
}

bool readRegisters(byte registerAddress, byte *buffer, byte length) {
  Wire.beginTransmission(ITG3205_ADDRESS);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ITG3205_ADDRESS, length) != length) return false;
  for (byte index = 0; index < length; index++) buffer[index] = Wire.read();
  return true;
}

bool setupGyro() {
  byte identity;
  if (!readRegisters(WHO_AM_I, &identity, 1)) return false;
  writeRegister(POWER_MANAGEMENT, 0x80);
  delay(10);
  writeRegister(SAMPLE_RATE_DIVIDER, 9);
  writeRegister(DLPF_FS, 0x1A);
  writeRegister(POWER_MANAGEMENT, 0x01);
  delay(10);
  return true;
}

bool readGyro(float &xDps, float &yDps, float &zDps) {
  byte data[6];
  if (!readRegisters(GYRO_XOUT_H, data, 6)) return false;
  int16_t xRaw = (int16_t)((data[0] << 8) | data[1]);
  int16_t yRaw = (int16_t)((data[2] << 8) | data[3]);
  int16_t zRaw = (int16_t)((data[4] << 8) | data[5]);
  xDps = xRaw / GYRO_SENSITIVITY;
  yDps = yRaw / GYRO_SENSITIVITY;
  zDps = zRaw / GYRO_SENSITIVITY;
  return true;
}

bool pressureAtLeast(int pressureRaw, int threshold) {
  return PRESSURE_INCREASES_WITH_SUCTION
    ? pressureRaw >= threshold
    : pressureRaw <= threshold;
}

bool pressureStrongerThanTargetBy(int pressureRaw, int margin) {
  return PRESSURE_INCREASES_WITH_SUCTION
    ? pressureRaw >= ADHESION_TARGET_RAW + margin
    : pressureRaw <= ADHESION_TARGET_RAW - margin;
}

void setBlowerPwm(int pwm) {
  blowerPwm = constrain(pwm, BLOWER_MIN_PWM, BLOWER_MAX_PWM);
  analogWrite(BLOWER_PWM_PIN, blowerPwm);
}

void updateAdhesionControl(int pressureRaw) {
  bool sensorFault =
    pressureRaw <= PRESSURE_SENSOR_MIN_RAW ||
    pressureRaw >= PRESSURE_SENSOR_MAX_RAW;

  if (sensorFault) {
    adhesionSecure = false;
    setBlowerPwm(BLOWER_MAX_PWM);
    return;
  }

  if (adhesionSecure) {
    if (!pressureAtLeast(pressureRaw, ADHESION_RELEASE_RAW)) {
      adhesionSecure = false;
    }
  } else if (pressureAtLeast(pressureRaw, ADHESION_TARGET_RAW)) {
    adhesionSecure = true;
  }

  if (!adhesionSecure) {
    setBlowerPwm(blowerPwm + CONTROL_STEP);
  } else if (pressureStrongerThanTargetBy(pressureRaw, CONTROL_RELAX_MARGIN_RAW)) {
    setBlowerPwm(blowerPwm - 1);
  }
}

void relayOn(int pin) {
  digitalWrite(pin, LOW);
}

void relayOff(int pin) {
  digitalWrite(pin, HIGH);
}

void stopDrive() {
  driveRunning = false;
  analogWrite(LEFT_DRIVE_PWM_PIN, 0);
  analogWrite(RIGHT_DRIVE_PWM_PIN, 0);
}

void setLeftDriveSpeed(int pwm) {
  leftDriveSpeed = constrain(pwm, 0, 255);
}

void setRightDriveSpeed(int pwm) {
  rightDriveSpeed = constrain(pwm, 0, 255);
}

void setDriveSpeeds(int leftPwm, int rightPwm) {
  setLeftDriveSpeed(leftPwm);
  setRightDriveSpeed(rightPwm);
}

void printDriveStatus() {
  Serial.print("STATUS,left_pwm=");
  Serial.print(leftDriveSpeed);
  Serial.print(",right_pwm=");
  Serial.print(rightDriveSpeed);
  Serial.print(",blower_pwm=");
  Serial.print(blowerPwm);
  Serial.print(",adhesion_secure=");
  Serial.println(adhesionSecure ? 1 : 0);
}

void runDrive() {
  if (!adhesionSecure) {
    stopDrive();
    Serial.println("DRIVE_BLOCKED,ADHESION_NOT_SECURE");
    return;
  }
  driveRunning = true;
  analogWrite(LEFT_DRIVE_PWM_PIN, leftDriveSpeed);
  analogWrite(RIGHT_DRIVE_PWM_PIN, rightDriveSpeed);
}

void driveForward() {
  stopDrive();
  delay(100);
  relayOn(LEFT_RELAY_1_PIN);
  relayOn(LEFT_RELAY_2_PIN);
  relayOff(RIGHT_RELAY_1_PIN);
  relayOff(RIGHT_RELAY_2_PIN);
  runDrive();
}

void driveBackward() {
  stopDrive();
  delay(100);
  relayOff(LEFT_RELAY_1_PIN);
  relayOff(LEFT_RELAY_2_PIN);
  relayOn(RIGHT_RELAY_1_PIN);
  relayOn(RIGHT_RELAY_2_PIN);
  runDrive();
}

void turnLeft() {
  stopDrive();
  delay(100);
  relayOff(LEFT_RELAY_1_PIN);
  relayOff(LEFT_RELAY_2_PIN);
  relayOff(RIGHT_RELAY_1_PIN);
  relayOff(RIGHT_RELAY_2_PIN);
  runDrive();
}

void turnRight() {
  stopDrive();
  delay(100);
  relayOn(LEFT_RELAY_1_PIN);
  relayOn(LEFT_RELAY_2_PIN);
  relayOn(RIGHT_RELAY_1_PIN);
  relayOn(RIGHT_RELAY_2_PIN);
  runDrive();
}

void processDriveCommand(char command) {
  switch (command) {
    case 'w': case 'W': driveForward(); break;
    case 's': case 'S': driveBackward(); break;
    case 'a': case 'A': turnLeft(); break;
    case 'd': case 'D': turnRight(); break;
    case 'x': case 'X': stopDrive(); break;
    case '?': printDriveStatus(); break;
    case '1': case '2': case '3':
    case '4': case '5': case '6':
    case '7': case '8': case '9': {
      int pwm = map(command - '0', 1, 9, 30, 255);
      setDriveSpeeds(pwm, pwm);
      if (driveRunning) runDrive();
      printDriveStatus();
      break;
    }
  }
}

void processDriveCommandLine(char *commandLine) {
  while (*commandLine == ' ' || *commandLine == '\t') commandLine++;
  if (*commandLine == '\0') return;

  char command = commandLine[0];
  if ((command == 'L' || command == 'l') && commandLine[1] != '\0') {
    setLeftDriveSpeed(atoi(commandLine + 1));
    if (driveRunning) runDrive();
    printDriveStatus();
    return;
  }
  if ((command == 'R' || command == 'r') && commandLine[1] != '\0') {
    setRightDriveSpeed(atoi(commandLine + 1));
    if (driveRunning) runDrive();
    printDriveStatus();
    return;
  }
  if ((command == 'P' || command == 'p') && commandLine[1] != '\0') {
    char *separator = strchr(commandLine + 1, ',');
    if (separator != NULL) {
      *separator = '\0';
      setDriveSpeeds(atoi(commandLine + 1), atoi(separator + 1));
      if (driveRunning) runDrive();
      printDriveStatus();
    } else {
      Serial.println("ERROR,USE_P_LEFT_COMMA_RIGHT");
    }
    return;
  }

  processDriveCommand(command);
}

void processSerialInput() {
  while (Serial.available()) {
    char incoming = Serial.read();
    if (incoming == '\n' || incoming == '\r') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processDriveCommandLine(commandBuffer);
        commandLength = 0;
      }
      continue;
    }

    if (commandLength < COMMAND_BUFFER_SIZE - 1) {
      commandBuffer[commandLength++] = incoming;
    } else {
      commandLength = 0;
      Serial.println("ERROR,COMMAND_TOO_LONG");
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(20);
  Wire.begin();
  pinMode(LEFT_RELAY_1_PIN, OUTPUT);
  pinMode(LEFT_RELAY_2_PIN, OUTPUT);
  pinMode(RIGHT_RELAY_1_PIN, OUTPUT);
  pinMode(RIGHT_RELAY_2_PIN, OUTPUT);
  pinMode(LEFT_DRIVE_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DRIVE_PWM_PIN, OUTPUT);
  pinMode(PRESSURE_PIN, INPUT);
  pinMode(BLOWER_PWM_PIN, OUTPUT);
  pinMode(BLOWER_ENABLE_PIN, OUTPUT);
  pinMode(BLOWER_DIRECTION_PIN, OUTPUT);

  // Timer2 prescaler=1. Arduino Uno D11 PWM becomes approximately 31.4 kHz,
  // within the blower driver's specified 2-50 kHz VSR input range.
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;
  digitalWrite(BLOWER_DIRECTION_PIN, HIGH);
  digitalWrite(BLOWER_ENABLE_PIN, HIGH);
  relayOff(LEFT_RELAY_1_PIN);
  relayOff(LEFT_RELAY_2_PIN);
  relayOff(RIGHT_RELAY_1_PIN);
  relayOff(RIGHT_RELAY_2_PIN);
  stopDrive();
  setBlowerPwm(BLOWER_START_PWM);
  startupTime = millis();
  gyroConnected = setupGyro();
}

void loop() {
  unsigned long now = millis();
  if (now - lastSensorTime < SENSOR_INTERVAL_MS) return;
  lastSensorTime = now;

  float gyroX = 0.0;
  float gyroY = 0.0;
  float gyroZ = 0.0;
  if (gyroConnected && !readGyro(gyroX, gyroY, gyroZ)) gyroConnected = false;
  int pressureRaw = analogRead(PRESSURE_PIN);
  updateAdhesionControl(pressureRaw);

  if (millis() - startupTime > STARTUP_GRACE_MS && !adhesionSecure) {
    // 주행 제어기는 이 상태를 수신하면 이동 모터를 구동하지 않아야 한다.
    setBlowerPwm(BLOWER_MAX_PWM);
  }
  if (!adhesionSecure && driveRunning) {
    stopDrive();
    Serial.println("EMERGENCY_DRIVE_STOP,ADHESION_LOST");
  }

  // SENSOR,millis,pressure_raw,gyro_valid,x_dps,y_dps,z_dps,blower_pwm,adhesion_secure
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
  Serial.print(gyroZ, 3);
  Serial.print(',');
  Serial.print(blowerPwm);
  Serial.print(',');
  Serial.println(adhesionSecure ? 1 : 0);

  processSerialInput();
}
