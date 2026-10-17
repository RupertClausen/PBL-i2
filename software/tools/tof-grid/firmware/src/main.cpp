// VL53L5CX 8x8 ToF -> USB serial.
// Each frame is one line:
//   F <d0> ... <d63> | <s0> ... <s63>
// d = distance in mm, s = target status (5 and 9 are valid ranges).
// Lines starting with '#' are informational.
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

constexpr int PIN_SDA = 8;
constexpr int PIN_SCL = 9;
constexpr int PIN_LPN = 6;  // low = I2C disabled, high = enabled
constexpr int PIN_INT = 5;  // open-drain, pulses low when a frame is ready
constexpr int RANGING_HZ = 15;  // 15 Hz is the maximum in 8x8 mode

SparkFun_VL53L5CX imager;
VL53L5CX_ResultsData results;
volatile bool frameReady = false;
uint32_t lastFrameMs = 0;

static void IRAM_ATTR onInt() { frameReady = true; }

static void haltWith(const char *msg) {
  while (true) {
    Serial.printf("# ERROR %s\n", msg);
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);  // give the host a moment to open the CDC port
  Serial.println("# tof-grid starting");

  // Cycle LPn to put the sensor's I2C interface into a known state
  pinMode(PIN_LPN, OUTPUT);
  digitalWrite(PIN_LPN, LOW);
  delay(10);
  digitalWrite(PIN_LPN, HIGH);
  delay(10);

  pinMode(PIN_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INT), onInt, FALLING);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(1000000);  // the ~86 KB sensor firmware upload is slow at 100 kHz

  Serial.println("# uploading sensor firmware (takes a few seconds)...");
  if (!imager.begin()) haltWith("VL53L5CX not found on I2C (addr 0x29) - check wiring/LPn");

  imager.setResolution(8 * 8);
  imager.setRangingFrequency(RANGING_HZ);
  frameReady = false;
  imager.startRanging();
  lastFrameMs = millis();
  Serial.println("# ranging 8x8");
}

void loop() {
  // INT drives reads; if it stays quiet (unwired/missed edge) fall back to polling
  bool ready = frameReady;
  if (!ready && millis() - lastFrameMs > 250) ready = imager.isDataReady();
  if (!ready) {
    delay(1);
    return;
  }
  frameReady = false;
  if (!imager.getRangingData(&results)) return;
  lastFrameMs = millis();

  char line[1024];
  int n = snprintf(line, sizeof line, "F");
  for (int i = 0; i < 64; i++) n += snprintf(line + n, sizeof line - n, " %d", results.distance_mm[i]);
  n += snprintf(line + n, sizeof line - n, " |");
  for (int i = 0; i < 64; i++) n += snprintf(line + n, sizeof line - n, " %u", results.target_status[i]);
  Serial.println(line);
}
