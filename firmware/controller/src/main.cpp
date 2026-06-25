// SPDX-FileCopyrightText: 2024 Kouhei Ito
// SPDX-FileCopyrightText: 2026 Atsushi Kataoka
// SPDX-License-Identifier: MIT
//
// ESP-NOW joystick transmitter for the Roller485 mecanum inverted pendulum.
// Sends a 25-byte broadcast packet on channel 3; pairing is intentionally skipped.
// The receiver does not start the motors until it receives a valid packet.
// For first bring-up, always test the receiver with the wheels lifted off the floor.

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <M5AtomS3.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>
#include <MPU6886.h>
#include <atoms3joy.h>
#include "buzzer.h"

#define CHANNEL 3

#define ANGLECONTROL 0
#define RATECONTROL 1
#define ANGLECONTROL_W_LOG 2
#define RATECONTROL_W_LOG 3
#define ALT_CONTROL_MODE 4
#define NOT_ALT_CONTROL_MODE 5
#define RESO10BIT (4096)

esp_now_peer_info_t peerInfo;

float Throttle;
float Phi, Theta, Psi;
uint16_t Phi_bias = 2048;
uint16_t Theta_bias = 2048;
uint16_t Psi_bias = 2048;
uint16_t Throttle_bias = 2048;
short xstick = 0;
short ystick = 0;
uint8_t Mode = ANGLECONTROL;
uint8_t AltMode = NOT_ALT_CONTROL_MODE;
volatile uint8_t Loop_flag = 0;
float Timer = 0.0;
float dTime = 0.01;
uint8_t Timer_state = 0;
uint8_t StickMode = 2;
uint32_t espnow_version;
volatile uint8_t proactive_flag = 0;
unsigned long stime, etime, dtime, ltime;
uint8_t axp_cnt = 0;
uint8_t senddata[25];
uint8_t disp_counter = 0;

uint8_t BroadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
volatile uint8_t Channel = CHANNEL;


#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "roller485-joy"
#endif

#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif

namespace {
constexpr uint32_t kOtaButtonHoldMs = 2000;
constexpr uint32_t kOtaWifiRetryIntervalMs = 10000;
constexpr size_t kEspNowPacketSize = 25;
constexpr uint8_t kEspNowChecksumOffset = 24;
constexpr uint8_t kPacketDestinationOffset = 0;
constexpr uint8_t kPsiOffset = 3;
constexpr uint8_t kThrottleOffset = 7;
constexpr uint8_t kPhiOffset = 11;
constexpr uint8_t kThetaOffset = 15;
bool otaModeEnabled = false;
bool otaStarted = false;
uint32_t lastOtaWifiAttemptMs = 0;
uint32_t otaButtonHoldStartMs = 0;

void writeFloatToPacket(uint8_t* packet, uint8_t offset, float value)
{
  memcpy(&packet[offset], &value, sizeof(value));
}

bool otaIsConfigured()
{
  return strlen(WIFI_SSID) > 0;
}

void setupOTA()
{
  if (otaStarted || !otaIsConfigured()) {
    return;
  }

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  if (strlen(OTA_PASSWORD) > 0) {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }

  ArduinoOTA
    .onStart([]() {
      Serial.println("OTA update started. ESP-NOW sending stopped.");
    })
    .onEnd([]() {
      Serial.println("\nOTA update finished.");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      const unsigned int percent = total == 0 ? 0 : (progress * 100) / total;
      static unsigned int lastPercent = 101;
      if (percent != lastPercent) {
        lastPercent = percent;
        Serial.printf("OTA progress: %u%%\r", percent);
      }
    })
    .onError([](ota_error_t error) {
      Serial.printf("OTA error[%u]\n", error);
    });

  ArduinoOTA.begin();
  otaStarted = true;
  M5.Lcd.fillScreen(BLUE);
  M5.Lcd.setCursor(4, 8);
  M5.Lcd.setTextColor(WHITE, BLUE);
  M5.Lcd.println("OTA READY");
  M5.Lcd.println(OTA_HOSTNAME);
  M5.Lcd.println(WiFi.localIP().toString());
  Serial.printf("OTA ready: %s.local IP: %s\n", OTA_HOSTNAME, WiFi.localIP().toString().c_str());
}

void enableOTAMode()
{
  if (otaModeEnabled) {
    return;
  }

  otaModeEnabled = true;
  esp_now_deinit();
  WiFi.disconnect(false, true);
  lastOtaWifiAttemptMs = 0;
  M5.Lcd.fillScreen(BLUE);
  M5.Lcd.setCursor(4, 8);
  M5.Lcd.setTextColor(WHITE, BLUE);
  M5.Lcd.println("OTA MODE");
  M5.Lcd.println("WiFi connecting");
  M5.Lcd.println(OTA_HOSTNAME);
  Serial.println("OTA mode enabled. Upload by espota.");
}

bool serviceOTAMode()
{
  if (!otaModeEnabled) {
    return false;
  }

  if (!otaIsConfigured()) {
    return true;
  }

  if (WiFi.status() != WL_CONNECTED) {
    const uint32_t now = millis();
    if (lastOtaWifiAttemptMs == 0 || now - lastOtaWifiAttemptMs >= kOtaWifiRetryIntervalMs) {
      lastOtaWifiAttemptMs = now;
      WiFi.mode(WIFI_STA);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      M5.Lcd.fillScreen(BLUE);
      M5.Lcd.setCursor(4, 8);
      M5.Lcd.setTextColor(WHITE, BLUE);
      M5.Lcd.println("OTA MODE");
      M5.Lcd.println("WiFi connecting");
      M5.Lcd.println(WIFI_SSID);
      Serial.printf("Connecting OTA WiFi: %s\n", WIFI_SSID);
    }
    return true;
  }

  setupOTA();
  ArduinoOTA.handle();
  return true;
}
}  // namespace

void rc_init(void)
{
  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESPNow Init Success");
  }
  else
  {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, BroadcastAddr, 6);
  peerInfo.channel = CHANNEL;
  peerInfo.encrypt = false;
  while (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
  }
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
}

// 周期カウンタ割り込み関数
hw_timer_t *timer = NULL;
void IRAM_ATTR onTimer()
{
  Loop_flag = 1;
}

void setup()
{
  M5.begin();
  Wire1.begin(38, 39, 400 * 1000);
  Serial1.begin(115200, SERIAL_8N1, 1, 2); // Grove atom_toio
  M5.update();
  setup_pwm_buzzer();
  M5.Lcd.setRotation(2);
  M5.Lcd.setTextFont(2);
  M5.Lcd.setCursor(4, 2);

  Channel = CHANNEL;
  rc_init();
  Serial.println("ESP-NOW broadcast mode. Pairing skipped.");
  M5.Lcd.fillScreen(BLACK);
  joy_update();

  StickMode = 2;
  if (getOptionButton())
  {
    StickMode = 3;
    M5.Lcd.println("Please release button.");
    while (getOptionButton())
      joy_update();
  }
  AltMode = NOT_ALT_CONTROL_MODE;
  delay(500);

  if (StickMode == 3)
  {
    THROTTLE = RIGHTY;
    AILERON = LEFTX;
    ELEVATOR = LEFTY;
    RUDDER = RIGHTX;
    ARM_BUTTON = RIGHT_STICK_BUTTON;
    FLIP_BUTTON = LEFT_STICK_BUTTON;
    MODE_BUTTON = RIGHT_BUTTON;
    OPTION_BUTTON = LEFT_BUTTON;
  }
  else
  {
    THROTTLE = LEFTY;
    AILERON = RIGHTX;
    ELEVATOR = RIGHTY;
    RUDDER = LEFTX;
    ARM_BUTTON = LEFT_STICK_BUTTON;
    FLIP_BUTTON = RIGHT_STICK_BUTTON;
    MODE_BUTTON = RIGHT_BUTTON;
    OPTION_BUTTON = LEFT_BUTTON;
  }

  byte error, address;
  int nDevices;

  Serial.println("Scanning... Wire1");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  esp_now_get_version(&espnow_version);
  Serial.printf("ESP-NOW Version %d\n", espnow_version);

  // 割り込み設定
  timer = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);
  delay(100);
}

uint8_t check_control_mode_change(void)
{
  uint8_t state;
  static uint8_t flag = 0;
  state = 0;
  if (flag == 0)
  {
    if (getModeButton() == 1)
    {
      flag = 1;
    }
  }
  else
  {
    if (getModeButton() == 0)
    {
      flag = 0;
      state = 1;
    }
  }
  return state;
}

uint8_t check_alt_mode_change(void)
{
  uint8_t state;
  static uint8_t flag = 0;
  state = 0;
  if (flag == 0)
  {
    if (getOptionButton() == 1)
    {
      flag = 1;
    }
  }
  else
  {
    if (getOptionButton() == 0)
    {
      flag = 0;
      state = 1;
    }
  }
  return state;
}

void loop()
{
  uint16_t _throttle;
  uint16_t _phi;
  uint16_t _theta;
  uint16_t _psi;

  ltime = micros() - stime;

  while (Loop_flag == 0)
    ;
  Loop_flag = 0;
  etime = stime;
  stime = micros();
  dtime = stime - etime;
  M5.update();
  joy_update();
  if (serviceOTAMode())
  {
    delay(1);
    return;
  }

  const bool otaButtonHeld = M5.Btn.isPressed() || getOptionButton();
  if (otaButtonHeld)
  {
    if (otaButtonHoldStartMs == 0)
    {
      otaButtonHoldStartMs = millis();
    }
    else if (millis() - otaButtonHoldStartMs >= kOtaButtonHoldMs)
    {
      enableOTAMode();
      delay(500);
      return;
    }
  }
  else
  {
    otaButtonHoldStartMs = 0;
  }


  // Stop Watch Start&Stop&Reset
  if (M5.Btn.wasPressed() == true)
  {
    if (Timer_state == 0)
      Timer_state = 1;
    else if (Timer_state == 1)
      Timer_state = 0;
  }

  if (M5.Btn.pressedFor(400) == true)
  {
    Timer_state = 2;
  }

  if (Timer_state == 1)
  {
    // カウントアップ
    Timer = Timer + dTime;
  }
  else if (Timer_state == 2)
  {
    // タイマリセット
    Timer = 0.0;
    Timer_state = 0;
  }

  if (check_control_mode_change() == 1)
  {
    if (Mode == ANGLECONTROL)
      Mode = RATECONTROL;
    else
      Mode = ANGLECONTROL;
  }

  if (check_alt_mode_change() == 1)
  {
    if (AltMode == ALT_CONTROL_MODE)
      AltMode = NOT_ALT_CONTROL_MODE;
    else
      AltMode = ALT_CONTROL_MODE;
  }

  _throttle = getThrottle();
  _phi = getAileron();
  _theta = getElevator();
  _psi = getRudder();

  if (getArmButton() == 1)
  {
    Phi_bias = _phi;
    Theta_bias = _theta;
    Psi_bias = _psi;
  }

  // 量産版
  Throttle = -(float)(_throttle - Throttle_bias) / (float)(RESO10BIT * 0.5);
  Phi = (float)(_phi - Phi_bias) / (float)(RESO10BIT * 0.5);
  Theta = (float)(_theta - Theta_bias) / (float)(RESO10BIT * 0.5);
  Psi = (float)(_psi - Psi_bias) / (float)(RESO10BIT * 0.5);

  // スティック補正
  Throttle = Throttle + 0.0;
  Phi = Phi + 0.0;
  Theta = Theta + 0.012;
  Psi = Psi - 0.047;

  senddata[kPacketDestinationOffset + 0] = 0xff;
  senddata[kPacketDestinationOffset + 1] = 0xff;
  senddata[kPacketDestinationOffset + 2] = 0xff;

  writeFloatToPacket(senddata, kPsiOffset, Psi);
  writeFloatToPacket(senddata, kThrottleOffset, Throttle);
  writeFloatToPacket(senddata, kPhiOffset, Phi);
  writeFloatToPacket(senddata, kThetaOffset, Theta);

  senddata[19] = getArmButton();
  senddata[20] = getFlipButton();
  senddata[21] = getModeButton();
  senddata[22] = getOptionButton();
  senddata[23] = proactive_flag;

  // checksum
  senddata[kEspNowChecksumOffset] = 0;
  for (uint8_t i = 0; i < kEspNowChecksumOffset; i++)
    senddata[kEspNowChecksumOffset] = senddata[kEspNowChecksumOffset] + senddata[i];

  // 送信
  esp_err_t result = esp_now_send(peerInfo.peer_addr, senddata, kEspNowPacketSize);
  (void)result;

  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextFont(0);
  M5.Lcd.setCursor(8, 5 + disp_counter * 16);
  switch (disp_counter)
  {
  case 0:
    M5.Lcd.printf("V1 %2.2f ", Battery_voltage[1]);
    break;
  case 1:
    M5.Lcd.printf("CH %d ", CHANNEL);
    break;
  case 2:
    M5.Lcd.printf("DT %2.1f ", (float)ltime / 1000);
    break;
  case 3:
    M5.Lcd.print("BC send ");
    break;
  }
  disp_counter++;
  if (disp_counter == 4)
    disp_counter = 0;

}
