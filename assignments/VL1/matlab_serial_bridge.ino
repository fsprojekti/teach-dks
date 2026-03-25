#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// Minimal firmware for reliable MATLAB serialRead/serialWrite communication.
// Protocol:
// - On boot: prints READY
// - Commands: run, stop, plot 1|0, pwm <0..4096>, dir <0|1>, ts <ms>
// - Stream (when plot=1): "angle1_deg angle2_deg vel1_rpm vel2_rpm" (exactly 4 numeric values)

// Motor pins
static const int PWM_PIN = 26;
static const int DIR_PIN = 25;
static const int PWM_FREQ = 10000;
static const int PWM_RES = 12;
static const int PWM_LIMIT = 4096;
static const int PWM_HW_MAX = (1 << PWM_RES) - 1;

// Encoder pins
static const int ENC1_A = 32;
static const int ENC1_B = 33;
static const int ENC2_A = 36;
static const int ENC2_B = 37;

// Encoder scaling
static const float ENC1_CPR = 4096.0f;
static const float ENC2_MOTOR_CPR = 48.0f;
static const float GEAR_RATIO = 20.4f;
static const float ENC2_OUT_CPR = ENC2_MOTOR_CPR * GEAR_RATIO;

// 2nd-order LPF target border frequency for velocity estimate.
// Internally clamped just below Nyquist for numerical stability.
static const float LPF_TARGET_HZ = 50.0f;
static const float LPF_Q = 0.70710678f; // Butterworth

// Shared state
volatile long enc1Count = 0;
volatile long enc2Count = 0;
volatile int enc1Prev = 0;
volatile int enc2Prev = 0;

volatile int runState = 0;
volatile int plotState = 0;
volatile int pwmCmd = 0;
volatile int dirCmd = 0;
volatile int tsMs = 10;

float vel1 = 0.0f;
float vel2 = 0.0f;

struct BiquadState {
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
  float z1;
  float z2;
};

BiquadState velLpf1 = {0};
BiquadState velLpf2 = {0};

void configureVelocityLpf(float fsHz) {
  if (fsHz < 1.0f) {
    fsHz = 1.0f;
  }

  float nyquist = 0.5f * fsHz;
  float fc = LPF_TARGET_HZ;
  float fcMaxStable = 0.49f * fsHz;
  if (fc >= nyquist) {
    fc = fcMaxStable;
  }
  if (fc > fcMaxStable) {
    fc = fcMaxStable;
  }
  if (fc < 0.01f) {
    fc = 0.01f;
  }

  float w0 = 2.0f * PI * fc / fsHz;
  float cosW0 = cosf(w0);
  float sinW0 = sinf(w0);
  float alpha = sinW0 / (2.0f * LPF_Q);

  float b0 = (1.0f - cosW0) * 0.5f;
  float b1 = 1.0f - cosW0;
  float b2 = (1.0f - cosW0) * 0.5f;
  float a0 = 1.0f + alpha;
  float a1 = -2.0f * cosW0;
  float a2 = 1.0f - alpha;

  // Normalize coefficients
  velLpf1.b0 = b0 / a0;
  velLpf1.b1 = b1 / a0;
  velLpf1.b2 = b2 / a0;
  velLpf1.a1 = a1 / a0;
  velLpf1.a2 = a2 / a0;

  velLpf2.b0 = velLpf1.b0;
  velLpf2.b1 = velLpf1.b1;
  velLpf2.b2 = velLpf1.b2;
  velLpf2.a1 = velLpf1.a1;
  velLpf2.a2 = velLpf1.a2;

  // Reset filter state after coefficient change.
  velLpf1.z1 = 0.0f;
  velLpf1.z2 = 0.0f;
  velLpf2.z1 = 0.0f;
  velLpf2.z2 = 0.0f;
}

float biquadStep(BiquadState& f, float x) {
  float y = f.b0 * x + f.z1;
  f.z1 = f.b1 * x - f.a1 * y + f.z2;
  f.z2 = f.b2 * x - f.a2 * y;
  return y;
}

TaskHandle_t samplingTaskHandle = nullptr;
TimerHandle_t samplingTimer = nullptr;

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

// Line buffer for command parser
char cmdBuf[96];
size_t cmdLen = 0;

long lastEnc1 = 0;
long lastEnc2 = 0;

void IRAM_ATTR enc1ISR() {
  int msb = digitalRead(ENC1_A);
  int lsb = digitalRead(ENC1_B);
  int encoded = (msb << 1) | lsb;
  int sum = (enc1Prev << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    enc1Count++;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    enc1Count--;
  }

  enc1Prev = encoded;
}

void IRAM_ATTR enc2ISR() {
  int msb = digitalRead(ENC2_A);
  int lsb = digitalRead(ENC2_B);
  int encoded = (msb << 1) | lsb;
  int sum = (enc2Prev << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    enc2Count++;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    enc2Count--;
  }

  enc2Prev = encoded;
}

void applyMotor() {
  int localRun;
  int localPwm;
  int localDir;

  portENTER_CRITICAL(&stateMux);
  localRun = runState;
  localPwm = pwmCmd;
  localDir = dirCmd;
  portEXIT_CRITICAL(&stateMux);

  if (localRun) {
    if (localPwm > PWM_HW_MAX) {
      localPwm = PWM_HW_MAX;
    }
    if (localPwm < 0) {
      localPwm = 0;
    }
    ledcWrite(PWM_PIN, localPwm);
    digitalWrite(DIR_PIN, localDir ? HIGH : LOW);
  } else {
    ledcWrite(PWM_PIN, 0);
    digitalWrite(DIR_PIN, LOW);
  }
}

void parseCommand(char* line) {
  while (*line == ' ' || *line == '\t') {
    line++;
  }
  if (*line == '\0') {
    return;
  }

  if (strcmp(line, "run") == 0) {
    portENTER_CRITICAL(&stateMux);
    runState = 1;
    portEXIT_CRITICAL(&stateMux);
    applyMotor();
    return;
  }

  if (strcmp(line, "stop") == 0) {
    portENTER_CRITICAL(&stateMux);
    runState = 0;
    portEXIT_CRITICAL(&stateMux);
    applyMotor();
    return;
  }

  int ival = 0;
  float fval = 0.0f;

  if (sscanf(line, "plot %d", &ival) == 1) {
    portENTER_CRITICAL(&stateMux);
    plotState = (ival > 0) ? 1 : 0;
    portEXIT_CRITICAL(&stateMux);
    return;
  }

  if (sscanf(line, "pwm %d", &ival) == 1) {
    if (ival < 0) {
      ival = 0;
    }
    if (ival > PWM_LIMIT) {
      ival = PWM_LIMIT;
    }
    portENTER_CRITICAL(&stateMux);
    pwmCmd = ival;
    portEXIT_CRITICAL(&stateMux);
    applyMotor();
    return;
  }

  if (sscanf(line, "dir %d", &ival) == 1) {
    portENTER_CRITICAL(&stateMux);
    dirCmd = (ival > 0) ? 1 : 0;
    portEXIT_CRITICAL(&stateMux);
    applyMotor();
    return;
  }

  if (sscanf(line, "ts %d", &ival) == 1) {
    if (ival >= 1 && ival <= 1000) {
      portENTER_CRITICAL(&stateMux);
      tsMs = ival;
      portEXIT_CRITICAL(&stateMux);

      if (samplingTimer != nullptr) {
        xTimerChangePeriod(samplingTimer, pdMS_TO_TICKS(ival), 0);
      }
    }
    return;
  }

  (void)fval;
}

void serialTask(void* pvParameters) {
  (void)pvParameters;

  while (1) {
    while (Serial.available() > 0) {
      char c = static_cast<char>(Serial.read());
      if (c == '\r' || c == '\n') {
        if (cmdLen > 0) {
          cmdBuf[cmdLen] = '\0';
          parseCommand(cmdBuf);
          cmdLen = 0;
        }
      } else {
        if (cmdLen < sizeof(cmdBuf) - 1) {
          cmdBuf[cmdLen++] = c;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void samplingTimerCallback(TimerHandle_t xTimer) {
  (void)xTimer;
  if (samplingTaskHandle != nullptr) {
    xTaskNotifyGive(samplingTaskHandle);
  }
}

void samplingTask(void* pvParameters) {
  (void)pvParameters;

  noInterrupts();
  lastEnc1 = enc1Count;
  lastEnc2 = enc2Count;
  interrupts();

  int prevTs = -1;

  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    long e1;
    long e2;
    noInterrupts();
    e1 = enc1Count;
    e2 = enc2Count;
    interrupts();

    int localTs;
    int localPlot;
    portENTER_CRITICAL(&stateMux);
    localTs = tsMs;
    localPlot = plotState;
    portEXIT_CRITICAL(&stateMux);

    if (localTs < 1) {
      localTs = 1;
    }

    if (localTs != prevTs) {
      float fs = 1000.0f / static_cast<float>(localTs);
      configureVelocityLpf(fs);
      prevTs = localTs;
    }

    long d1 = e1 - lastEnc1;
    long d2 = e2 - lastEnc2;
    lastEnc1 = e1;
    lastEnc2 = e2;

    float dtSec = static_cast<float>(localTs) / 1000.0f;
    float rawVel1 = (static_cast<float>(d1) / ENC1_CPR) / dtSec;      // turn/s
    float rawVel2 = (static_cast<float>(d2) / ENC2_OUT_CPR) / dtSec;  // turn/s

    vel1 = biquadStep(velLpf1, rawVel1);
    vel2 = biquadStep(velLpf2, rawVel2);

    if (localPlot) {
      // MATLAB expects exactly 4 numeric values per line.
      float angle1Deg = (static_cast<float>(e1) * 360.0f) / ENC1_CPR;
      float angle2Deg = (static_cast<float>(e2) * 360.0f) / ENC2_OUT_CPR;
      float vel1Rpm = vel1 * 60.0f;
      float vel2Rpm = vel2 * 60.0f;

      Serial.print(angle1Deg, 6);
      Serial.print(' ');
      Serial.print(angle2Deg, 6);
      Serial.print(' ');
      Serial.print(vel1Rpm, 6);
      Serial.print(' ');
      Serial.println(vel2Rpm, 6);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2);

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  enc1Prev = (digitalRead(ENC1_A) << 1) | digitalRead(ENC1_B);
  enc2Prev = (digitalRead(ENC2_A) << 1) | digitalRead(ENC2_B);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), enc2ISR, CHANGE);

  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM_PIN, 0);

  noInterrupts();
  lastEnc1 = enc1Count;
  lastEnc2 = enc2Count;
  interrupts();

  xTaskCreatePinnedToCore(serialTask, "serialTask", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(samplingTask, "samplingTask", 4096, nullptr, 2, &samplingTaskHandle, 1);

  samplingTimer = xTimerCreate(
    "samplingTimer",
    pdMS_TO_TICKS(tsMs),
    pdTRUE,
    nullptr,
    samplingTimerCallback
  );
  if (samplingTimer != nullptr) {
    xTimerStart(samplingTimer, 0);
  }

  Serial.println("READY");
}

void loop() {
  // Runtime handled by FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
