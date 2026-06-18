// 6847A pressure sensor standalone monitor
// Wiring:
//   6847A VCC -> Arduino 5V
//   6847A GND -> Arduino GND
//   6847A OUT -> Arduino A0
// Serial Monitor:
//   Baud: 115200
//   Line ending: any

const unsigned long SERIAL_BAUD = 115200;
const unsigned long SAMPLE_INTERVAL_MS = 1000;
const int PRINT_ONLY_IF_RAW_CHANGED_BY = 0;  // 0 = always print every interval
const int PRESSURE_PIN = A0;
const int AVERAGE_SAMPLES = 16;
const float ADC_REFERENCE_VOLTAGE = 5.0;
const int ADC_MAX = 1023;

unsigned long lastSampleTime = 0;
int lastPrintedRaw = -1;

int readPressureAverageRaw() {
  long total = 0;
  for (int index = 0; index < AVERAGE_SAMPLES; index++) {
    total += analogRead(PRESSURE_PIN);
    delayMicroseconds(300);
  }
  return (int)(total / AVERAGE_SAMPLES);
}

float rawToVoltage(int raw) {
  return raw * ADC_REFERENCE_VOLTAGE / ADC_MAX;
}

void printHeader() {
  Serial.println("6847A_PRESSURE_MONITOR_READY");
  Serial.println("CSV,millis,raw,voltage_v");
  Serial.println("INFO,prints once per second by default");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(PRESSURE_PIN, INPUT);
  delay(500);
  printHeader();
}

void loop() {
  unsigned long now = millis();
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = now;

  int pressureRaw = readPressureAverageRaw();
  float pressureVoltage = rawToVoltage(pressureRaw);

  if (PRINT_ONLY_IF_RAW_CHANGED_BY > 0 && lastPrintedRaw >= 0 &&
      abs(pressureRaw - lastPrintedRaw) < PRINT_ONLY_IF_RAW_CHANGED_BY) {
    return;
  }

  lastPrintedRaw = pressureRaw;
  Serial.print("PRESSURE,");
  Serial.print(now);
  Serial.print(',');
  Serial.print(pressureRaw);
  Serial.print(',');
  Serial.println(pressureVoltage, 3);
}
