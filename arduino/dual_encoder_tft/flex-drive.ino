#include <Arduino.h>
#include <TFT_eSPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// =========================
// FlexDrive minimal firmware (FreeRTOS)
// MATLAB-compatible protocol:
//   - READY on boot
//   - commands: run, stop, plot 1/0, pwm <0..4000>, dir <0|1>, ts <ms>, alpha <0..1>
//   - when plot=1: exactly 4 values per line:
//       enc1Pos enc2Pos enc1Vel enc2Vel
// =========================

// Motor control
static const int PWM_PIN = 26;
static const int DIR_PIN = 25;
static const int PWM_FREQUENCY = 10000;
static const int PWM_RESOLUTION = 12;
static const int PWM_MAX = 4000;

// Encoder pins
static const int ENC1_A = 32;
static const int ENC1_B = 33;
static const int ENC2_A = 36;
static const int ENC2_B = 37;

// Encoder scaling
static const float ENC1_COUNTS_PER_REV = 4096.0f;
static const float ENC2_COUNTS_PER_MOTOR_REV = 48.0f;
static const float ENC2_GEAR_RATIO = 20.4f;
static const float ENC2_COUNTS_PER_OUTPUT_REV = ENC2_COUNTS_PER_MOTOR_REV * ENC2_GEAR_RATIO;

// TFT refresh
static const uint32_t TFT_UPDATE_MS = 120;

// Quadrature decoder state
volatile long encoder1Position = 0;
volatile long encoder2Position = 0;
volatile int lastEncoded1 = 0;
volatile int lastEncoded2 = 0;

// Shared runtime state
volatile int pwmCommand = 0;
volatile int dirCommand = 0;
volatile int runState = 0;       // 0=stop, 1=run
volatile int plotEnabled = 0;    // 0=off, 1=on
volatile int tsMs = 10;          // sampling time
volatile float alpha = 0.05f;    // velocity filter

volatile float encoder1Velocity = 0.0f; // turn/s (output shaft)
volatile float encoder2Velocity = 0.0f; // turn/s (output shaft equivalent)

TFT_eSPI tft = TFT_eSPI();

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR encoder1ISR() {
  int msb = digitalRead(ENC1_A);
  int lsb = digitalRead(ENC1_B);
  int encoded = (msb << 1) | lsb;
  int sum = (lastEncoded1 << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoder1Position++;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoder1Position--;
  }

  lastEncoded1 = encoded;
}

void IRAM_ATTR encoder2ISR() {
  int msb = digitalRead(ENC2_A);
  int lsb = digitalRead(ENC2_B);
  int encoded = (msb << 1) | lsb;
  int sum = (lastEncoded2 << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoder2Position++;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoder2Position--;
  }

  lastEncoded2 = encoded;
}

void applyMotorOutput() {
  int localRun;
  int localPwm;
  int localDir;

  portENTER_CRITICAL(&stateMux);
  localRun = runState;
  localPwm = pwmCommand;
  localDir = dirCommand;
  portEXIT_CRITICAL(&stateMux);

  if (localRun == 1) {
    ledcWrite(PWM_PIN, localPwm);
    digitalWrite(DIR_PIN, localDir ? HIGH : LOW);
  } else {
    ledcWrite(PWM_PIN, 0);
    digitalWrite(DIR_PIN, LOW);
  }
}

void handleCommand(const String& cmdLine) {
  String line = cmdLine;
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line.equalsIgnoreCase("run")) {
    portENTER_CRITICAL(&stateMux);
    runState = 1;
    portEXIT_CRITICAL(&stateMux);
    applyMotorOutput();
    return;
  }

  if (line.equalsIgnoreCase("stop")) {
    portENTER_CRITICAL(&stateMux);
    runState = 0;
    portEXIT_CRITICAL(&stateMux);
    applyMotorOutput();
    return;
  }

  if (line.startsWith("plot ")) {
    int val = line.substring(5).toInt();
    portENTER_CRITICAL(&stateMux);
    plotEnabled = (val > 0) ? 1 : 0;
    portEXIT_CRITICAL(&stateMux);
    return;
  }

  if (line.startsWith("pwm ")) {
    int val = line.substring(4).toInt();
    if (val < 0) val = 0;
    if (val > PWM_MAX) val = PWM_MAX;
    portENTER_CRITICAL(&stateMux);
    pwmCommand = val;
    portEXIT_CRITICAL(&stateMux);
    applyMotorOutput();
    return;
  }

  if (line.startsWith("dir ")) {
    int val = line.substring(4).toInt();
    portENTER_CRITICAL(&stateMux);
    dirCommand = (val > 0) ? 1 : 0;
    portEXIT_CRITICAL(&stateMux);
    applyMotorOutput();
    return;
  }

  if (line.startsWith("ts ")) {
    int val = line.substring(3).toInt();
    if (val >= 1) {
      portENTER_CRITICAL(&stateMux);
      tsMs = val;
      portEXIT_CRITICAL(&stateMux);
    }
    return;
  }

  if (line.startsWith("alpha ")) {
    float val = line.substring(6).toFloat();
    if (val >= 0.0f && val <= 1.0f) {
      portENTER_CRITICAL(&stateMux);
      alpha = val;
      portEXIT_CRITICAL(&stateMux);
    }
    return;
  }

  if (line.equalsIgnoreCase("list")) {
    int localRun;
    int localPlot;
    int localPwm;
    int localDir;
    int localTs;
    float localAlpha;

    portENTER_CRITICAL(&stateMux);
    localRun = runState;
    localPlot = plotEnabled;
    localPwm = pwmCommand;
    localDir = dirCommand;
    localTs = tsMs;
    localAlpha = alpha;
    portEXIT_CRITICAL(&stateMux);

    Serial.printf("status %d %d %d %d %d %.3f\n", localRun, localPlot, localPwm, localDir, localTs, localAlpha);
    return;
  }
}

void serialTask(void* pvParameters) {
  (void)pvParameters;
  String line;
  while (1) {
    while (Serial.available() > 0) {
      char c = static_cast<char>(Serial.read());
      if (c == '\n' || c == '\r') {
        if (line.length() > 0) {
          handleCommand(line);
          line = "";
        }
      } else {
        line += c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void samplingTask(void* pvParameters) {
  (void)pvParameters;

  long lastEnc1 = 0;
  long lastEnc2 = 0;

  portENTER_CRITICAL(&encoderMux);
  lastEnc1 = encoder1Position;
  lastEnc2 = encoder2Position;
  portEXIT_CRITICAL(&encoderMux);

  uint32_t lastTickMs = millis();

  while (1) {
    int localTs;
    float localAlpha;
    int localPlot;

    portENTER_CRITICAL(&stateMux);
    localTs = tsMs;
    localAlpha = alpha;
    localPlot = plotEnabled;
    portEXIT_CRITICAL(&stateMux);

    vTaskDelay(pdMS_TO_TICKS(localTs));

    uint32_t nowMs = millis();
    float dt = (nowMs - lastTickMs) / 1000.0f;
    if (dt <= 0.0f) {
      dt = localTs / 1000.0f;
    }
    lastTickMs = nowMs;

    long enc1;
    long enc2;
    portENTER_CRITICAL(&encoderMux);
    enc1 = encoder1Position;
    enc2 = encoder2Position;
    portEXIT_CRITICAL(&encoderMux);

    long d1 = enc1 - lastEnc1;
    long d2 = enc2 - lastEnc2;
    lastEnc1 = enc1;
    lastEnc2 = enc2;

    float sampleFreq = 1.0f / dt;
    float rawVel1 = (d1 / ENC1_COUNTS_PER_REV) * sampleFreq;
    float rawVel2 = (d2 / ENC2_COUNTS_PER_OUTPUT_REV) * sampleFreq;

    portENTER_CRITICAL(&stateMux);
    encoder1Velocity = localAlpha * rawVel1 + (1.0f - localAlpha) * encoder1Velocity;
    encoder2Velocity = localAlpha * rawVel2 + (1.0f - localAlpha) * encoder2Velocity;
    float outVel1 = encoder1Velocity;
    float outVel2 = encoder2Velocity;
    portEXIT_CRITICAL(&stateMux);

    if (localPlot == 1) {
      // IMPORTANT: exactly 4 numeric values for MATLAB parser.
      Serial.print(enc1);
      Serial.print(' ');
      Serial.print(enc2);
      Serial.print(' ');
      Serial.print(outVel1, 6);
      Serial.print(' ');
      Serial.println(outVel2, 6);
    }
  }
}

void tftTask(void* pvParameters) {
  (void)pvParameters;

  while (1) {
    long enc1;
    long enc2;
    int localRun;
    int localPlot;
    int localPwm;
    int localDir;
    int localTs;
    float localAlpha;
    float localVel1;
    float localVel2;

    portENTER_CRITICAL(&encoderMux);
    enc1 = encoder1Position;
    enc2 = encoder2Position;
    portEXIT_CRITICAL(&encoderMux);

    portENTER_CRITICAL(&stateMux);
    localRun = runState;
    localPlot = plotEnabled;
    localPwm = pwmCommand;
    localDir = dirCommand;
    localTs = tsMs;
    localAlpha = alpha;
    localVel1 = encoder1Velocity;
    localVel2 = encoder2Velocity;
    portEXIT_CRITICAL(&stateMux);

    tft.fillRect(0, 24, 320, 216, TFT_BLACK);
    tft.setCursor(0, 24);
    tft.printf("run:%d  plot:%d\n", localRun, localPlot);
    tft.printf("pwm:%d  dir:%d\n", localPwm, localDir);
    tft.printf("ts:%dms alpha:%.3f\n\n", localTs, localAlpha);

    tft.println("ENC1 (output)");
    tft.printf("pos:%ld\n", enc1);
    tft.printf("vel:%.4f turn/s\n\n", localVel1);

    tft.println("ENC2 (output-eq)");
    tft.printf("pos:%ld\n", enc2);
    tft.printf("vel:%.4f turn/s\n", localVel2);

    vTaskDelay(pdMS_TO_TICKS(TFT_UPDATE_MS));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2);

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  lastEncoded1 = (digitalRead(ENC1_A) << 1) | digitalRead(ENC1_B);
  lastEncoded2 = (digitalRead(ENC2_A) << 1) | digitalRead(ENC2_B);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), encoder2ISR, CHANGE);

  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  ledcAttach(PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcWrite(PWM_PIN, 0);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("FlexDrive Debug");
  tft.println("MATLAB serial mode");

  xTaskCreatePinnedToCore(serialTask, "serialTask", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(samplingTask, "samplingTask", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(tftTask, "tftTask", 4096, nullptr, 1, nullptr, 0);

  Serial.println("READY");
}

void loop() {
  // All runtime logic handled in FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
