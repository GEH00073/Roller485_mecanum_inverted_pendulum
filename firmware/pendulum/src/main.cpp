// SPDX-FileCopyrightText: 2024 Kouhei Ito
// SPDX-FileCopyrightText: 2026 Atsushi Kataoka
// SPDX-License-Identifier: MIT
//
// Experimental firmware for a mecanum-wheel inverted pendulum using four Roller485 units.
// The 5 ms control loop updates the IMU, estimates attitude, runs balance control, and
// distributes current commands to the four wheels.
// For first bring-up, always test with the wheels lifted off the floor.
// Motors are stopped on ESP-NOW receiver timeout or fall detection.
// Motors do not start until a valid ESP-NOW packet is received; local button-only
// startup is intentionally disabled for safety.

#include <Arduino.h>
#include <ArduinoOTA.h>
#include "unit_rolleri2c.hpp"
#include <M5Unified.h>
#include <MadgwickAHRS.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>

// Roller layout reference:
//   LEFT   = left outer,  addr 0x64
//   LEFT2  = left inner,  addr 0x65
//   RIGHT2 = right inner, addr 0x66
//   RIGHT  = right outer, addr 0x67
UnitRollerI2C RollerI2C_RIGHT;  // Create a UNIT_ROLLERI2C object
UnitRollerI2C RollerI2C_RIGHT2;
UnitRollerI2C RollerI2C_LEFT;
UnitRollerI2C RollerI2C_LEFT2;
//I2Cドライバの多重初期化を防ぐためのフラグ
bool UnitRollerI2C::initialized = false;

float Pitch_ahrs, Roll_ahrs, Yaw_ahrs, Roll_bias, Roll;
float Gyro_x, Gyro_y, Gyro_z;
float Acc_x, Acc_y, Acc_z;
int32_t Imu_time,_Imu_time, Imu_dtime;
int32_t Current_ref_r, Current_ref_l;
int32_t Current_ref_r2, Current_ref_l2;
int32_t Current_r, Current_l;
int32_t Current_r2, Current_l2;
int32_t Pos_r, Pos_l, Pos_bias_r, Pos_bias_l;
int32_t Pos_r2, Pos_l2, Pos_bias_r2, Pos_bias_l2;
int32_t Speed_r, Speed_l;
int32_t Speed_r2, Speed_l2;
int32_t Voltage_r,Voltage_l;
int32_t Voltage_r2,Voltage_l2;
int32_t St, _St, Et, Dt;
float f1,f2,f3,f4;
float k1;
float U0, U_yaw, U_v, U_l;
uint8_t Start_flag = 0;
bool mode_flag = false;
float yaw, angle, start_yaw, target_yaw, diff;

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "roller485-pendulum"
#endif

#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif

namespace {
constexpr uint32_t kStartupOtaWindowMs = 5000;
constexpr int32_t kBatteryDetectVinRaw = 600;  // getVin() is displayed as raw / 100.0 V.
constexpr int32_t kStartupCheckCurrent = 10000;
constexpr uint32_t kStartupCheckPulseMs = 450;
constexpr uint32_t kStartupCheckSettleMs = 250;
constexpr uint32_t kControlPeriodMs = 5;
constexpr uint32_t kRcTimeoutMs = 300;
constexpr float kFallStopAngleDeg = 70.0f;
constexpr int32_t kCurrentLimit = 120000;
constexpr float kStickDeadband = 0.05f;
constexpr float kRollAngleGain = 7000.0f;
constexpr float kGyroGain = 250.0f;
constexpr float kWheelSpeedGain = -5.0f;
constexpr float kYawRateGain = 300.0f;
constexpr float kForwardGain = 150000.0f;
constexpr float kLateralGain = 100000.0f;
constexpr float kLateralYawCompensationGain = 67000.0f;
constexpr float kYawReferenceScale = -360.0f / 2.0f;
constexpr float kDriftTargetYawStep = 1.0f;
constexpr uint32_t kUiToggleDebounceMs = 500;
constexpr uint32_t kRollBiasAdjustIntervalMs = 100;
constexpr float kRollBiasStepDeg = 0.2f;
constexpr uint8_t kEspNowChannel = 3;
constexpr uint8_t kBroadcastAddr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
bool startupOtaStarted = false;
bool startupOtaInProgress = false;
volatile uint8_t MyMacAddr[6];
volatile uint8_t Rc_err_flag = 0;
volatile uint8_t Recv_MAC[3];
volatile uint16_t Connect_flag = 0;
volatile float Stick[16];
volatile uint32_t last_rc_rx_ms = 0;
portMUX_TYPE stickMux = portMUX_INITIALIZER_UNLOCKED;
esp_now_peer_info_t peerInfo;

enum StickIndex : uint8_t {
    RUDDER = 0,
    ELEVATOR = 1,
    THROTTLE = 2,
    AILERON = 3,
    LOG = 4,
    BUTTON_ARM = 9,
    BUTTON_FLIP = 10,
    CONTROLMODE = 11,
    ALTCONTROLMODE = 12,
};

void copyStickSnapshot(float (&snapshot)[16])
{
    portENTER_CRITICAL(&stickMux);
    for (uint8_t i = 0; i < 16; i++) {
        snapshot[i] = Stick[i];
    }
    portEXIT_CRITICAL(&stickMux);
}

uint32_t getLastRcRxMs()
{
    portENTER_CRITICAL(&stickMux);
    const uint32_t lastRxMs = last_rc_rx_ms;
    portEXIT_CRITICAL(&stickMux);
    return lastRxMs;
}

float readFloatFromPacket(const uint8_t* recvData, uint8_t offset)
{
    float value = 0.0f;
    memcpy(&value, &recvData[offset], sizeof(value));
    return value;
}

int32_t clampCurrent(int32_t current)
{
    if (current > kCurrentLimit) {
        return kCurrentLimit;
    }
    if (current < -kCurrentLimit) {
        return -kCurrentLimit;
    }
    return current;
}

struct StartupRollerCheckStep {
    const char* label;
    uint8_t addr;
    UnitRollerI2C* roller;
    int8_t logicalSign;
};

bool startupOtaIsConfigured()
{
    return strlen(WIFI_SSID) > 0;
}

void onEspNowRecv(const uint8_t* macAddr, const uint8_t* recvData, int dataLen)
{
    (void)macAddr;
    Connect_flag = 0;

    if (dataLen < 25) {
        Rc_err_flag = 1;
        return;
    }

    Recv_MAC[0] = recvData[0];
    Recv_MAC[1] = recvData[1];
    Recv_MAC[2] = recvData[2];

    const bool addressedToMe =
        (recvData[0] == MyMacAddr[3]) && (recvData[1] == MyMacAddr[4]) && (recvData[2] == MyMacAddr[5]);
    const bool addressedToBroadcast =
        (recvData[0] == 0xff) && (recvData[1] == 0xff) && (recvData[2] == 0xff);

    if (!addressedToMe && !addressedToBroadcast) {
        Rc_err_flag = 1;
        return;
    }

    uint8_t checkSum = 0;
    for (uint8_t i = 0; i < 24; i++) {
        checkSum = checkSum + recvData[i];
    }
    if (checkSum != recvData[24]) {
        Rc_err_flag = 1;
        return;
    }

    Rc_err_flag = 0;

    const float rudder = readFloatFromPacket(recvData, 3);
    const float throttle = readFloatFromPacket(recvData, 7);
    const float aileron = readFloatFromPacket(recvData, 11);
    const float elevator = readFloatFromPacket(recvData, 15);
    const uint32_t nowMs = millis();

    portENTER_CRITICAL(&stickMux);
    Stick[RUDDER] = rudder;
    Stick[THROTTLE] = throttle;
    Stick[AILERON] = aileron;
    Stick[ELEVATOR] = elevator;
    Stick[BUTTON_ARM] = recvData[19];
    Stick[BUTTON_FLIP] = recvData[20];
    Stick[CONTROLMODE] = recvData[21];
    Stick[ALTCONTROLMODE] = recvData[22];
    Stick[LOG] = 0.0f;
    last_rc_rx_ms = nowMs;
    portEXIT_CRITICAL(&stickMux);
}

void rcInit()
{
    portENTER_CRITICAL(&stickMux);
    for (uint8_t i = 0; i < 16; i++) {
        Stick[i] = 0.0f;
    }
    last_rc_rx_ms = 0;
    portEXIT_CRITICAL(&stickMux);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    WiFi.macAddress((uint8_t*)MyMacAddr);
    Serial.printf("MAC ADDRESS: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                  MyMacAddr[0], MyMacAddr[1], MyMacAddr[2],
                  MyMacAddr[3], MyMacAddr[4], MyMacAddr[5]);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESPNow Init Failed");
        ESP.restart();
    }
    Serial.println("ESPNow Init Success");

    esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, kBroadcastAddr, 6);
    peerInfo.channel = kEspNowChannel;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add broadcast peer");
        return;
    }

    esp_now_register_recv_cb(onEspNowRecv);
    Serial.println("ESP-NOW Broadcast Ready.");
}

void setupStartupOTA()
{
    if (startupOtaStarted || !startupOtaIsConfigured()) {
        return;
    }

    ArduinoOTA.setHostname(OTA_HOSTNAME);
    if (strlen(OTA_PASSWORD) > 0) {
        ArduinoOTA.setPassword(OTA_PASSWORD);
    }

    ArduinoOTA
        .onStart([]() {
            startupOtaInProgress = true;
            Start_flag = 0;
            Serial.println("OTA update started during startup window.");
            M5.Display.clear(TFT_BLUE);
            M5.Display.setCursor(0, 10);
            M5.Display.print("OTA");
        })
        .onEnd([]() {
            Serial.println("\nOTA update finished. Restarting.");
            delay(500);
            ESP.restart();
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
            startupOtaInProgress = false;
            Serial.printf("OTA error[%u]\n", error);
        });

    ArduinoOTA.begin();
    startupOtaStarted = true;
    Serial.printf("Startup OTA ready: %s.local IP: %s\n", OTA_HOSTNAME, WiFi.localIP().toString().c_str());
}

void updateImuDuringStartupOTA()
{
    M5.Imu.update();
    auto imudata = M5.Imu.getImuData();
    Gyro_x = imudata.gyro.x;
    Gyro_y = imudata.gyro.y;
    Gyro_z = imudata.gyro.z;
    Acc_x = imudata.accel.x;
    Acc_y = imudata.accel.y;
    Acc_z = imudata.accel.z;
    MadgwickAHRSupdateIMU(Gyro_x * DEG_TO_RAD, Gyro_y * DEG_TO_RAD, Gyro_z * DEG_TO_RAD, Acc_x, Acc_y, Acc_z, &Pitch_ahrs, &Roll_ahrs, &Yaw_ahrs);
    Roll = 0.3f * Roll + 0.7f * Roll_ahrs;
}

void serviceStartupOTAWindow()
{
    if (!startupOtaIsConfigured()) {
        return;
    }

    const uint32_t windowStartMs = millis();
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("Startup OTA + IMU calibration window: connecting WiFi %s\n", WIFI_SSID);
    M5.Display.clear(TFT_BLUE);
    M5.Display.setCursor(0, 10);
    M5.Display.print("OTA+IMU");

    while ((millis() - windowStartMs < kStartupOtaWindowMs) || startupOtaInProgress) {
        updateImuDuringStartupOTA();
        if (WiFi.status() == WL_CONNECTED) {
            setupStartupOTA();
            ArduinoOTA.handle();
        }
        M5.update();
        delay(10);
    }

    if (startupOtaStarted) {
        ArduinoOTA.end();
        startupOtaStarted = false;
    }
    WiFi.disconnect(false, true);
    WiFi.mode(WIFI_OFF);
    M5.Display.clear(TFT_BLACK);
    Serial.println("Startup OTA window closed. Continuing normal startup.");
}

void stopAllRollers()
{
    Current_ref_r = 0;
    Current_ref_l = 0;
    Current_ref_r2 = 0;
    Current_ref_l2 = 0;
    RollerI2C_RIGHT.setCurrent(0);
    RollerI2C_LEFT.setCurrent(0);
    RollerI2C_RIGHT2.setCurrent(0);
    RollerI2C_LEFT2.setCurrent(0);
}

void stopInitializedRollers(bool rightOk, bool leftOk, bool right2Ok, bool left2Ok)
{
    Current_ref_r = 0;
    Current_ref_l = 0;
    Current_ref_r2 = 0;
    Current_ref_l2 = 0;
    if (rightOk) {
        RollerI2C_RIGHT.setCurrent(0);
    }
    if (leftOk) {
        RollerI2C_LEFT.setCurrent(0);
    }
    if (right2Ok) {
        RollerI2C_RIGHT2.setCurrent(0);
    }
    if (left2Ok) {
        RollerI2C_LEFT2.setCurrent(0);
    }
}

void waitWithM5Update(uint32_t waitMs)
{
    const uint32_t startMs = millis();
    while (millis() - startMs < waitMs) {
        M5.update();
        delay(10);
    }
}

int32_t readStartupBatteryVinRaw()
{
    int32_t vin = RollerI2C_RIGHT.getVin();
    vin = max(vin, RollerI2C_LEFT.getVin());
    vin = max(vin, RollerI2C_RIGHT2.getVin());
    vin = max(vin, RollerI2C_LEFT2.getVin());
    return vin;
}

void applyLogicalCurrent(const StartupRollerCheckStep& step, int32_t current)
{
    step.roller->setCurrent(step.logicalSign * current);
}

void showStartupCheckScreen(const char* label, uint8_t addr, int32_t vinRaw)
{
    M5.Display.clear(TFT_BLACK);
    M5.Display.setCursor(0, 4);
    M5.Display.printf("CHK %s\n", label);
    M5.Display.printf("addr 0x%02X\n", addr);
    M5.Display.printf("Vin %4.1fV\n", (float)vinRaw / 100.0f);
}

void runStartupRollerCheckMode()
{
    const int32_t vinRaw = readStartupBatteryVinRaw();
    if (vinRaw < kBatteryDetectVinRaw) {
        Serial.printf("Startup roller check skipped. Vin %.2f V\n", (float)vinRaw / 100.0f);
        return;
    }

    const StartupRollerCheckStep steps[] = {
        {"LEFT",   0x64, &RollerI2C_LEFT,   -1},
        {"LEFT2",  0x65, &RollerI2C_LEFT2,   1},
        {"RIGHT2", 0x66, &RollerI2C_RIGHT2, -1},
        {"RIGHT",  0x67, &RollerI2C_RIGHT,   1},
    };

    Serial.printf("Startup roller check started. Vin %.2f V\n", (float)vinRaw / 100.0f);
    M5.Display.clear(TFT_BLUE);
    M5.Display.setCursor(0, 4);
    M5.Display.println("ROLLER CHECK");
    M5.Display.printf("Vin %4.1fV\n", (float)vinRaw / 100.0f);
    waitWithM5Update(800);

    stopAllRollers();
    for (const auto& step : steps) {
        showStartupCheckScreen(step.label, step.addr, vinRaw);
        Serial.printf("Check %s addr=0x%02X logical current=%ld raw current=%ld\n",
                      step.label,
                      step.addr,
                      (long)kStartupCheckCurrent,
                      (long)(step.logicalSign * kStartupCheckCurrent));
        applyLogicalCurrent(step, kStartupCheckCurrent);
        waitWithM5Update(kStartupCheckPulseMs);
        stopAllRollers();
        waitWithM5Update(kStartupCheckSettleMs);
    }

    M5.Display.clear(TFT_BLUE);
    M5.Display.setCursor(0, 4);
    M5.Display.println("CHK FWD");
    M5.Display.println("all wheels");
    Serial.println("Check all wheels logical forward");
    for (const auto& step : steps) {
        applyLogicalCurrent(step, kStartupCheckCurrent);
    }
    waitWithM5Update(kStartupCheckPulseMs);
    stopAllRollers();
    waitWithM5Update(kStartupCheckSettleMs);

    M5.Display.clear(TFT_BLACK);
    M5.Display.setCursor(0, 4);
    M5.Display.println("CHECK DONE");
    Serial.println("Startup roller check finished.");
    waitWithM5Update(500);
}
}  // namespace

void uiTask(void *pvParameters) {
    uint32_t lastStartToggleMs = 0;
    uint32_t lastModeToggleMs = 0;
    uint32_t lastRollBiasAdjustMs = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(50));
        float stick[16];
        copyStickSnapshot(stick);
        const uint32_t nowMs = millis();

        M5.Display.setCursor(0, 10);
        M5.Display.printf("roll %4.1f\n", Roll);
        M5.Display.printf("V    %4.1f\n", (float)Voltage_r/100.0);
        M5.Display.printf("dt   %4.1f\n", (float)Dt/1.0e3);
        M5.Display.printf("yaw  %4.1f\n", (float)target_yaw);
        M5.Display.printf("diff %4.1f\n", (float)diff);

        M5.update();
        if((stick[CONTROLMODE] > 0.9 || M5.BtnA.wasPressed()) &&
           (nowMs - lastStartToggleMs >= kUiToggleDebounceMs)){ //右ボタンでトグル
            //Star_flagをトグル 
            Start_flag = !Start_flag;
            Roll_bias = Roll;
            Pos_bias_r = Pos_r;
            Pos_bias_l = Pos_l;
            start_yaw = -Yaw_ahrs;
            target_yaw = 0; 
            lastStartToggleMs = nowMs;
        }

        f4 = kWheelSpeedGain;

        if(stick[ALTCONTROLMODE] > 0.9 &&
           (nowMs - lastModeToggleMs >= kUiToggleDebounceMs)){ //左ボタンでトグル
            mode_flag = !mode_flag; //true = drift mode, false = normal mode
            if(mode_flag == true){
                start_yaw = -Yaw_ahrs;
                target_yaw = 0; 
            }
            lastModeToggleMs = nowMs;
        }

        if(nowMs - lastRollBiasAdjustMs >= kRollBiasAdjustIntervalMs) {
            if(stick[THROTTLE] > 0.9){ //THROTTLE上でRoll_bias UP
                Roll_bias = Roll_bias - kRollBiasStepDeg;
                lastRollBiasAdjustMs = nowMs;
            }
            else if(stick[THROTTLE] < -0.9){ //THROTTLE下でRoll_bias Down
                Roll_bias = Roll_bias + kRollBiasStepDeg;
                lastRollBiasAdjustMs = nowMs;
            }
        }

        if(Roll < -kFallStopAngleDeg or Roll > kFallStopAngleDeg){ //転倒したらOFF
            Start_flag = 0;
        }

    }
}

void controlTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(kControlPeriodMs); // 5ms の周期

    // 初期化
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // 次の周期まで待機
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        _St = St;
        St = micros();
        float stick[16];
        copyStickSnapshot(stick);
        const uint32_t lastRcRxMs = getLastRcRxMs();
        const bool rcTimedOut = (lastRcRxMs == 0) || (millis() - lastRcRxMs > kRcTimeoutMs);

        // TODO: If M5Unified exposes a reliable IMU update failure status for this board,
        // stopAllRollers() should be called on failure.
        M5.Imu.update();
        auto imudata = M5.Imu.getImuData();
        Gyro_x = imudata.gyro.x;
        Gyro_y = imudata.gyro.y;
        Gyro_z = imudata.gyro.z;
        Acc_x = imudata.accel.x;
        Acc_y = imudata.accel.y;
        Acc_z = imudata.accel.z;
        
        MadgwickAHRSupdateIMU(Gyro_x * DEG_TO_RAD, Gyro_y * DEG_TO_RAD, Gyro_z * DEG_TO_RAD, Acc_x, Acc_y, Acc_z, &Pitch_ahrs, &Roll_ahrs, &Yaw_ahrs);
        k1 = 0.3;
        Roll = k1*Roll +(1-k1)*Roll_ahrs;

        Current_r =  RollerI2C_RIGHT.getCurrentReadback();
        Current_r2 =  -RollerI2C_RIGHT2.getCurrentReadback();
        Current_l = -RollerI2C_LEFT.getCurrentReadback();
        Current_l2 = RollerI2C_LEFT2.getCurrentReadback();
        Pos_r =  RollerI2C_RIGHT.getPosReadback();
        Pos_r2 =  -RollerI2C_RIGHT2.getPosReadback();
        Pos_l = -RollerI2C_LEFT.getPosReadback();
        Pos_l2 = RollerI2C_LEFT2.getPosReadback();
        Speed_r =  RollerI2C_RIGHT.getSpeedReadback();
        Speed_r2 =  -RollerI2C_RIGHT2.getSpeedReadback();
        Speed_l = -RollerI2C_LEFT.getSpeedReadback();
        Speed_l2 = RollerI2C_LEFT2.getSpeedReadback();
        Voltage_r = RollerI2C_RIGHT.getVin();
        //Voltage_l = RollerI2C_LEFT.getVin();
        // TODO: UnitRollerI2C::getErrorCode() exists, but the exact error semantics are
        // not documented here. Add stopAllRollers() when a reliable fault condition is defined.

        if (rcTimedOut) {
            Start_flag = 0;
        }

        // current control
        if(Start_flag==1){
            f1 = kRollAngleGain; //振子の角度に比例して電流を制御
            f2 = kGyroGain; //振子の角速度に比例して電流を制御
            float a = stick[RUDDER];
            float b = -stick[ELEVATOR];
            float c = stick[AILERON];
            if( a > -kStickDeadband and a < kStickDeadband){a = 0.0;} //dead band
            if( b > -kStickDeadband and b < kStickDeadband){b = 0.0;} //dead band
            if( c > -kStickDeadband and c < kStickDeadband){c = 0.0;} //dead band

            if(mode_flag == true){ //drift mode
                float angle_rad = target_yaw * PI / 180.0;
                // 回転計算
                float x1 = c * cos(angle_rad) - b * sin(angle_rad);
                float y1 = c * sin(angle_rad) + b * cos(angle_rad);
                c = x1 / 2.0;
                b = y1;

                target_yaw += a * kDriftTargetYawStep;
                if(target_yaw > 360) target_yaw -= 360;
                if(target_yaw < 0) target_yaw += 360;
                diff = -Yaw_ahrs - start_yaw - target_yaw;
                if(diff > 180) diff -= 360;
                if(diff < -180) diff += 360;
                a = -diff / 10;
            }

            float yaw_ref = kYawReferenceScale * a; 
            float yaw_err = yaw_ref - Gyro_z;
            U_yaw = yaw_err * kYawRateGain; //回転
            U_v = -b * kForwardGain; //前後移動 ELEVATOR
            U_l = c * kLateralGain; //左右移動 AILERON
            float C_yaw = -c * kLateralYawCompensationGain; //左右移動時の回転モーメント補正
            //State feedback control
            U0 = (-f1 * (Roll - Roll_bias) 
                 - f2 * Gyro_x 
                 - f3 * (float)(Pos_r - Pos_bias_r + Pos_l - Pos_bias_l)/2.0 
                 - f4 * (Speed_r + Speed_l + Speed_r2 + Speed_l2)/4.0 );

            Current_ref_r  = (int32_t)(U0 + U_v + U_yaw + U_l + C_yaw);
            Current_ref_r2 = (int32_t)(U0 + U_v + U_yaw - U_l); 
            Current_ref_l  = (int32_t)(U0 + U_v - U_yaw - U_l - C_yaw);
            Current_ref_l2 = (int32_t)(U0 + U_v - U_yaw + U_l);

            //limit current
            Current_ref_r = clampCurrent(Current_ref_r);
            Current_ref_l = clampCurrent(Current_ref_l);
            Current_ref_r2 = clampCurrent(Current_ref_r2);
            Current_ref_l2 = clampCurrent(Current_ref_l2);

            RollerI2C_RIGHT.setCurrent(Current_ref_r);
            RollerI2C_RIGHT2.setCurrent(-Current_ref_r2);
            RollerI2C_LEFT.setCurrent(-Current_ref_l);
            RollerI2C_LEFT2.setCurrent(Current_ref_l2);
        }
        else{
            stopAllRollers();
        }
        Et = micros();
        Dt = Et - St;

    }
}

void setup(){

    Roll_bias = 0.0;
    auto cfg = M5.config();     
    M5.begin(cfg);
    M5.Display.setTextSize(2);               // テキストサイズを変更
    serviceStartupOTAWindow();
    rcInit();
    delay(1000);
    printf("Start\n");
    f3 = 0.0f;
    f4 = kWheelSpeedGain;

    const bool rollerRightOk = RollerI2C_RIGHT.begin(0x67, 8, 7, 400000);
    if(rollerRightOk == true){
        printf("RollerI2C_RIGHT begin success\n");
    }
    else{
        printf("RollerI2C_RIGHT begin failed\n");
    }
    const bool rollerLeftOk = RollerI2C_LEFT.begin(0x64, 8, 7, 400000);
    if(rollerLeftOk == true){
        printf("RollerI2C_LEFT begin success\n");
    }
    else{
        printf("RollerI2C_LEFT begin failed\n");
    }
    const bool rollerRight2Ok = RollerI2C_RIGHT2.begin(0x66, 8, 7, 400000);
    if(rollerRight2Ok == true){
        printf("RollerI2C_RIGHT2 begin success\n");
    }
    else{
        printf("RollerI2C_RIGHT2 begin failed\n");
    }
    const bool rollerLeft2Ok = RollerI2C_LEFT2.begin(0x65, 8, 7, 400000);
    if(rollerLeft2Ok == true){
        printf("RollerI2C_LEFT2 begin success\n");
    }
    else{
        printf("RollerI2C_LEFT2 begin failed\n");
    }

    if (!rollerRightOk || !rollerLeftOk || !rollerRight2Ok || !rollerLeft2Ok) {
        Start_flag = 0;
        stopInitializedRollers(rollerRightOk, rollerLeftOk, rollerRight2Ok, rollerLeft2Ok);
        M5.Display.clear(TFT_RED);
        M5.Display.setCursor(0, 10);
        M5.Display.println("Roller init");
        M5.Display.println("failed");
        Serial.println("Roller init failed. Halt.");
        while (true) {
            M5.update();
            delay(100);
        }
    }

    RollerI2C_RIGHT.setMode(3);
    RollerI2C_LEFT.setMode(3);
    RollerI2C_RIGHT2.setMode(3);
    RollerI2C_LEFT2.setMode(3);

    stopAllRollers();
    RollerI2C_RIGHT.setOutput(1);
    RollerI2C_LEFT.setOutput(1);
    RollerI2C_RIGHT2.setOutput(1);
    RollerI2C_LEFT2.setOutput(1);

    runStartupRollerCheckMode();


    // FreeRTOSタスクの作成
    BaseType_t result = xTaskCreateUniversal(
        controlTask,
        "Control 5ms Task",
        8192,
        NULL,
        5,
        NULL,
        APP_CPU_NUM
    );

    if (result != pdPASS) {
        printf("Task creation failed: %d\n", result);
        while (1); // 無限ループで停止
    }

    result = xTaskCreateUniversal(
        uiTask,
        "UI Display Task",
        8192,
        NULL,
        1,
        NULL,
        APP_CPU_NUM
    );

    if (result != pdPASS) {
        printf("Task creation failed: %d\n", result);
        while (1); // 無限ループで停止
    }
}

void loop() {
    // FreeRTOSを使用する場合、loop関数は空にします
    delay(1);
}
