/*
========================================================
 ESP32-S3 TRANSMITTER + BMI270 (SparkFun lib)
 DVS / Phase DIY - ESP-NOW low latency motion packet
 Versao Otimizada V2: Corrigido erro de compilação
========================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <math.h>
#include "SparkFun_BMI270_Arduino_Library.h"

#define DECK_ID 1

#define ESPNOW_CHANNEL 11
#define SEND_RATE_HZ 500
#define SEND_INTERVAL_US (1000000UL / SEND_RATE_HZ)
#define HANDSHAKE_INTERVAL_MS 250
#define HANDSHAKE_TIMEOUT_MS 2000

#define PROTOCOL_VERSION 1
#define MSG_HELLO 1
#define MSG_WELCOME 2
#define MSG_DATA 3
#define MSG_PING 4

// Ajuste os pinos conforme a fiacao do seu LED RGB no S3.
#define LED_R 4
#define LED_G 5
#define LED_B 6
#define LED_COMMON_ANODE 0
#define LED_BLINK_MS 50 

uint8_t receiverMAC[] = { 0x14, 0xC1, 0x9F, 0x2C, 0xDE, 0x7C };

// Filtro assimetrico otimizado para menor latencia
float ALPHA_SLOW = 0.40f; 
float ALPHA_FAST = 0.85f; 
float FAST_THRESHOLD_RPM = 2.0f;
float DEADZONE_RPM = 0.15f;
float RPM_MULTIPLIER = 1.00f;

// Auto-calibracao acelerada
#define AUTO_CALIBRATION_STABLE_SAMPLES 150 
#define AUTO_CALIBRATION_SAMPLE_DELAY_MS 1   
#define AUTO_CALIBRATION_STABLE_DPS_DELTA 2.5f
#define AUTO_CALIBRATION_MIN_COMPARE_SAMPLES 10

typedef struct __attribute__((packed)) {
  uint8_t msgType;
  uint8_t version;
  uint8_t deckId;
  int16_t rpmCenti;
  int16_t gyroRaw;
  uint32_t seq;
  uint32_t timestampMicros;
} dvs_packet;

dvs_packet packet;

volatile bool receiverReady = false;
volatile uint32_t lastReceiverReplyMillis = 0;

BMI270 imu;

float gyroOffsetZ = 0.0f;
float filteredRPM = 0.0f;
uint32_t sequenceNumber = 0;
uint32_t nextSendMicros = 0;

static inline uint8_t ledOnLevel() {
  return LED_COMMON_ANODE ? LOW : HIGH;
}

static inline uint8_t ledOffLevel() {
  return LED_COMMON_ANODE ? HIGH : LOW;
}

void setLED(bool red, bool green, bool blue) {
  digitalWrite(LED_R, red ? ledOnLevel() : ledOffLevel());
  digitalWrite(LED_G, green ? ledOnLevel() : ledOffLevel());
  digitalWrite(LED_B, blue ? ledOnLevel() : ledOffLevel());
}

void setupLED() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setLED(false, false, false);
}

void blinkLED(bool red, bool green, bool blue, uint16_t times) {
  for (uint16_t i = 0; i < times; i++) {
    setLED(red, green, blue);
    delay(LED_BLINK_MS);
    setLED(false, false, false);
    delay(LED_BLINK_MS);
  }
}

bool probeBMI270() {
  if (imu.beginI2C(BMI2_I2C_SEC_ADDR) == BMI2_OK) return true;
  if (imu.beginI2C(BMI2_I2C_PRIM_ADDR) == BMI2_OK) return true;
  return false;
}

void setupBMI270() {
  Wire.begin();
  Wire.setClock(400000); 

  int attempts = 0;
  while (!probeBMI270()) {
    Serial.println("BMI270 nao encontrado!");
    setLED(true, false, false);
    delay(100);
    setLED(false, false, false);
    delay(100);
    attempts++;
    if (attempts > 10) break; 
  }
  Serial.println("BMI270 Inicializado.");
}

static inline bool readGyroZDps(float *dps) {
  imu.getSensorData();
  *dps = imu.data.gyroZ;
  return true;
}

// FUNÇÃO QUE ESTAVA FALTANDO E CAUSAVA O ERRO:
static inline float dpsToRPM(float dps) {
  float corrected = dps - gyroOffsetZ;
  float rpm = -(corrected / 6.0f) * RPM_MULTIPLIER;
  return rpm;
}

void autoCalibrateGyroZ() {
  int stableSamples = 0;
  float sum = 0.0f;
  uint32_t startCal = millis();

  Serial.println("Calibrando...");
  while (stableSamples < AUTO_CALIBRATION_STABLE_SAMPLES) {
    float dps = 0.0f;
    readGyroZDps(&dps);

    if (stableSamples >= AUTO_CALIBRATION_MIN_COMPARE_SAMPLES) {
      float mean = sum / stableSamples;
      if (fabsf(dps - mean) > AUTO_CALIBRATION_STABLE_DPS_DELTA) {
        stableSamples = 0;
        sum = 0.0f;
        if (millis() - startCal > 2000) break; 
        continue;
      }
    }

    sum += dps;
    stableSamples++;
    delay(AUTO_CALIBRATION_SAMPLE_DELAY_MS);
  }

  if (stableSamples > 0) {
    gyroOffsetZ = sum / (float)stableSamples;
  }
  Serial.println("Calibrado.");
}

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *dataPtr, int len) {
  if (len != sizeof(dvs_packet)) return;
  dvs_packet incoming;
  memcpy(&incoming, dataPtr, sizeof(incoming));
  if (incoming.version != PROTOCOL_VERSION || incoming.deckId != DECK_ID) return;
  if (incoming.msgType == MSG_WELCOME || incoming.msgType == MSG_PING) {
    receiverReady = true;
    lastReceiverReplyMillis = millis();
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) return;

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void sendControlMessage(uint8_t msgType) {
  dvs_packet control = {};
  control.msgType = msgType;
  control.version = PROTOCOL_VERSION;
  control.deckId = DECK_ID;
  control.seq = sequenceNumber;
  control.timestampMicros = micros();
  esp_now_send(receiverMAC, (uint8_t *)&control, sizeof(control));
}

void setup() {
  Serial.begin(115200);
  
  setupLED();
  setupBMI270();
  setupEspNow();
  
  sendControlMessage(MSG_HELLO);
  
  autoCalibrateGyroZ();
  setLED(false, true, false);
  nextSendMicros = micros();
}

void loop() {
  uint32_t now = micros();
  
  static uint32_t lastHello = 0;
  if (!receiverReady && (millis() - lastHello > HANDSHAKE_INTERVAL_MS)) {
    sendControlMessage(MSG_HELLO);
    lastHello = millis();
  }

  if ((int32_t)(now - nextSendMicros) < 0) return;

  nextSendMicros += SEND_INTERVAL_US;
  if ((int32_t)(now - nextSendMicros) > (int32_t)SEND_INTERVAL_US) {
    nextSendMicros = now + SEND_INTERVAL_US;
  }

  float dps = 0.0f;
  readGyroZDps(&dps);
  float rpm = dpsToRPM(dps);

  float delta = rpm - filteredRPM;
  float alpha = (fabsf(delta) > FAST_THRESHOLD_RPM) ? ALPHA_FAST : ALPHA_SLOW;
  filteredRPM += delta * alpha;

  if (fabsf(filteredRPM) < DEADZONE_RPM) filteredRPM = 0.0f;

  packet.msgType = MSG_DATA;
  packet.version = PROTOCOL_VERSION;
  packet.deckId = DECK_ID;
  packet.rpmCenti = (int16_t)constrain(lroundf(filteredRPM * 100.0f), -32768, 32767);
  packet.gyroRaw = (int16_t)constrain(lroundf(dps * 10.0f), -32768, 32767);
  packet.seq = sequenceNumber++;
  packet.timestampMicros = now;

  esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));

  if (receiverReady && (millis() - lastReceiverReplyMillis > HANDSHAKE_TIMEOUT_MS)) {
    receiverReady = false;
    Serial.println("Conexao perdida...");
  }
}
