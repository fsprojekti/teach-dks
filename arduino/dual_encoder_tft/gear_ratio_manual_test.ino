#include <Arduino.h>

// Encoder pins
static const int PIN_MAG_A = 32;   // Output shaft magnetic encoder A
static const int PIN_MAG_B = 33;   // Output shaft magnetic encoder B
static const int PIN_OPT_A = 36;   // Motor-side optical encoder A
static const int PIN_OPT_B = 37;   // Motor-side optical encoder B

// Encoder scaling assumptions
static const float MAG_COUNTS_PER_REV = 4096.0f;     // Output shaft
static const float OPT_COUNTS_PER_MOTOR_REV = 48.0f; // Motor shaft

// Stop condition for manual test (target 10 output revolutions)
static const int32_t TARGET_OUTPUT_REVS = 10;
static const int32_t TARGET_MAG_COUNTS = static_cast<int32_t>(MAG_COUNTS_PER_REV * TARGET_OUTPUT_REVS);
static const uint32_t PRINT_INTERVAL_MS = 200;

// Quadrature transition table
// index = (prev_state << 2) | new_state
// state = (A << 1) | B
static const int8_t QEM[16] = {
  0, -1, +1,  0,
  +1, 0,  0, -1,
  -1, 0,  0, +1,
  0, +1, -1,  0
};

volatile int32_t magCount = 0;
volatile int32_t optCount = 0;
volatile uint8_t magPrevState = 0;
volatile uint8_t optPrevState = 0;

int32_t startMag = 0;
int32_t startOpt = 0;
bool testDone = false;

void IRAM_ATTR updateMagEncoder() {
  uint8_t a = static_cast<uint8_t>(digitalRead(PIN_MAG_A));
  uint8_t b = static_cast<uint8_t>(digitalRead(PIN_MAG_B));
  uint8_t state = static_cast<uint8_t>((a << 1) | b);
  uint8_t idx = static_cast<uint8_t>((magPrevState << 2) | state);
  magCount += QEM[idx];
  magPrevState = state;
}

void IRAM_ATTR updateOptEncoder() {
  uint8_t a = static_cast<uint8_t>(digitalRead(PIN_OPT_A));
  uint8_t b = static_cast<uint8_t>(digitalRead(PIN_OPT_B));
  uint8_t state = static_cast<uint8_t>((a << 1) | b);
  uint8_t idx = static_cast<uint8_t>((optPrevState << 2) | state);
  optCount += QEM[idx];
  optPrevState = state;
}

void IRAM_ATTR isrMagA() { updateMagEncoder(); }
void IRAM_ATTR isrMagB() { updateMagEncoder(); }
void IRAM_ATTR isrOptA() { updateOptEncoder(); }
void IRAM_ATTR isrOptB() { updateOptEncoder(); }

void setup() {
  Serial.begin(115200);

  pinMode(PIN_MAG_A, INPUT_PULLUP);
  pinMode(PIN_MAG_B, INPUT_PULLUP);
  pinMode(PIN_OPT_A, INPUT_PULLUP);
  pinMode(PIN_OPT_B, INPUT_PULLUP);

  magPrevState = static_cast<uint8_t>((digitalRead(PIN_MAG_A) << 1) | digitalRead(PIN_MAG_B));
  optPrevState = static_cast<uint8_t>((digitalRead(PIN_OPT_A) << 1) | digitalRead(PIN_OPT_B));

  attachInterrupt(digitalPinToInterrupt(PIN_MAG_A), isrMagA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_MAG_B), isrMagB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_OPT_A), isrOptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_OPT_B), isrOptB, CHANGE);

  delay(200);

  noInterrupts();
  startMag = magCount;
  startOpt = optCount;
  interrupts();

  Serial.println("[GEAR TEST] READY");
  Serial.println("[GEAR TEST] Manually rotate output shaft by 10 full turns.");
  Serial.printf("[GEAR TEST] Test auto-finishes when magnetic encoder reaches ~%ld counts.\n",
                static_cast<long>(TARGET_MAG_COUNTS));
}

void loop() {
  static uint32_t lastPrintMs = 0;

  if (testDone) {
    return;
  }

  int32_t magNow;
  int32_t optNow;
  noInterrupts();
  magNow = magCount;
  optNow = optCount;
  interrupts();

  int32_t deltaMag = magNow - startMag;
  int32_t deltaOpt = optNow - startOpt;

  int32_t absMag = (deltaMag >= 0) ? deltaMag : -deltaMag;

  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;

    float progressPct = (100.0f * absMag) / TARGET_MAG_COUNTS;
    if (progressPct > 999.0f) progressPct = 999.0f;

    Serial.printf("[GEAR TEST] progress: mag=%ld / %ld (%.1f%%), opt=%ld\n",
                  static_cast<long>(deltaMag),
                  static_cast<long>(TARGET_MAG_COUNTS),
                  progressPct,
                  static_cast<long>(deltaOpt));
  }

  if (absMag >= TARGET_MAG_COUNTS) {
    float outputRevs = absMag / MAG_COUNTS_PER_REV;
    float motorRevs = ((deltaOpt >= 0) ? deltaOpt : -deltaOpt) / OPT_COUNTS_PER_MOTOR_REV;

    float estimatedGearRatio = 0.0f;
    if (outputRevs > 0.0f) {
      estimatedGearRatio = motorRevs / outputRevs;
    }

    float angleDeg = (deltaMag * 360.0f) / MAG_COUNTS_PER_REV;

    Serial.println("[GEAR TEST] DONE");
    Serial.printf("[GEAR TEST] deltaMagCounts = %ld\n", static_cast<long>(deltaMag));
    Serial.printf("[GEAR TEST] deltaMagAngleDeg = %.2f\n", angleDeg);
    Serial.printf("[GEAR TEST] deltaOptCounts = %ld\n", static_cast<long>(deltaOpt));
    Serial.printf("[GEAR TEST] outputRevs = %.5f\n", outputRevs);
    Serial.printf("[GEAR TEST] motorRevs = %.5f\n", motorRevs);
    Serial.printf("[GEAR TEST] estimatedGearRatio (motor:output) = %.4f:1\n", estimatedGearRatio);
    Serial.println("[GEAR TEST] Reset board to repeat.");

    testDone = true;
  }
}
