#include <Arduino.h>

// Encoder pins (magnetic encoder only)
static const int PIN_MAG_A = 32;
static const int PIN_MAG_B = 33;

// Motor drive pins
static const int PIN_PWM = 26;
static const int PIN_DIR = 25;
static const int PWM_FREQUENCY = 10000;
static const int PWM_RESOLUTION = 12;
static const int PWM_MAX = (1 << PWM_RESOLUTION) - 1;
static const int DIR_FORWARD_LEVEL = HIGH;

// Encoder scaling
static const float MAG_COUNTS_PER_REV = 4096.0f;

// Test timing
static const uint32_t TEST_START_DELAY_MS = 1000;
static const uint32_t TEST_DURATION_MS = 10000;

// Quadrature decode
static const int8_t QEM[16] = {
  0, -1, +1,  0,
  +1, 0,  0, -1,
  -1, 0,  0, +1,
  0, +1, -1,  0
};

enum TestState {
  TEST_WAIT_START,
  TEST_RUNNING,
  TEST_DONE
};

volatile int32_t magCount = 0;
volatile uint8_t magPrevState = 0;

TestState testState = TEST_WAIT_START;
uint32_t testStateStartMs = 0;
int32_t testStartMagCount = 0;
int32_t testEndMagCount = 0;
float testRpmFromAngle = 0.0f;
float testDeltaAngleDeg = 0.0f;

void IRAM_ATTR updateMagEncoder() {
  uint8_t a = static_cast<uint8_t>(digitalRead(PIN_MAG_A));
  uint8_t b = static_cast<uint8_t>(digitalRead(PIN_MAG_B));
  uint8_t state = static_cast<uint8_t>((a << 1) | b);
  uint8_t idx = static_cast<uint8_t>((magPrevState << 2) | state);
  magCount += QEM[idx];
  magPrevState = state;
}

void IRAM_ATTR isrMagA() { updateMagEncoder(); }
void IRAM_ATTR isrMagB() { updateMagEncoder(); }

void setup() {
  Serial.begin(115200);

  pinMode(PIN_MAG_A, INPUT_PULLUP);
  pinMode(PIN_MAG_B, INPUT_PULLUP);

  magPrevState = static_cast<uint8_t>((digitalRead(PIN_MAG_A) << 1) | digitalRead(PIN_MAG_B));

  attachInterrupt(digitalPinToInterrupt(PIN_MAG_A), isrMagA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_MAG_B), isrMagB, CHANGE);

  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_DIR, LOW);
  ledcAttach(PIN_PWM, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcWrite(PIN_PWM, 0);

  testStateStartMs = millis();
  Serial.println("[TEST] READY: wait 1 s, then run 100% PWM for 10 s");
}

void loop() {
  static bool donePrinted = false;

  uint32_t nowMs = millis();

  if (testState == TEST_WAIT_START && (nowMs - testStateStartMs >= TEST_START_DELAY_MS)) {
    noInterrupts();
    testStartMagCount = magCount;
    interrupts();

    digitalWrite(PIN_DIR, DIR_FORWARD_LEVEL);
    ledcWrite(PIN_PWM, PWM_MAX);

    testState = TEST_RUNNING;
    testStateStartMs = nowMs;

    Serial.println("[TEST] START: 100% PWM for 10 s");
  }

  if (testState == TEST_RUNNING && (nowMs - testStateStartMs >= TEST_DURATION_MS)) {
    ledcWrite(PIN_PWM, 0);

    noInterrupts();
    testEndMagCount = magCount;
    interrupts();

    int32_t deltaMag = testEndMagCount - testStartMagCount;
    testDeltaAngleDeg = (deltaMag * 360.0f) / MAG_COUNTS_PER_REV;
    float durationMin = TEST_DURATION_MS / 60000.0f;
    testRpmFromAngle = (testDeltaAngleDeg / 360.0f) / durationMin;

    if (testRpmFromAngle < 0.0f) {
      testRpmFromAngle = -testRpmFromAngle;
    }

    testState = TEST_DONE;

    Serial.printf("[TEST] DONE: deltaCount=%ld deltaAngleDeg=%.2f rpm_from_angle=%.2f\n",
                  static_cast<long>(deltaMag),
                  testDeltaAngleDeg,
                  testRpmFromAngle);

    donePrinted = false;
  }

  if (testState == TEST_DONE && !donePrinted) {
    donePrinted = true;
    Serial.println("[TEST] COMPLETE. Reset board to run again.");
  }
}
