#include <Arduino.h>
#include "unit_rolleri2c.hpp"
#include <M5Unified.h>
#include <MadgwickAHRS.h>
#include <rc.hpp>
#include <telemetry.hpp>

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
//TaskHandle_t xHandle = NULL;

void dummyTask(void *pvParameters) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(50));
        //printf("time:%6.3fms roll:%7.3f pitch:%7.3f yaw:%7.3f C_r:%7d C_l:%7d P_r:%8.3f P_l:%8.3f S_r:%8.3f S_l:%8.3f\n",
        //(float)(Dt)/1.0e3, Roll_ahrs, Pitch_ahrs, Yaw_ahrs,
        //Current_r, Current_l, (float)Pos_r/100.0, (float)Pos_l/100.0, (float)Speed_r/100.0, (float)Speed_l/100.0);

        M5.Display.setCursor(0, 10);
        M5.Display.printf("roll %4.1f\n", Roll);
        M5.Display.printf("V    %4.1f\n", (float)Voltage_r/100.0);
        M5.Display.printf("dt   %4.1f\n", (float)Dt/1.0e3);
        M5.Display.printf("yaw  %4.1f\n", (float)target_yaw);
        M5.Display.printf("diff %4.1f\n", (float)diff);

        M5.update();
        // if(M5.BtnA.wasPressed())
        if(Stick[CONTROLMODE] > 0.9 || M5.BtnA.wasPressed()){ //右ボタンでトグル
            //Star_flagをトグル 
            Start_flag = !Start_flag;
            Roll_bias = Roll;
            Pos_bias_r = Pos_r;
            Pos_bias_l = Pos_l;
            start_yaw = -Yaw_ahrs;
            target_yaw = 0; 
            delay(500); //チャタリング防止
        }

        f4 = -5.0; //-3.0;

        if(Stick[ALTCONTROLMODE] > 0.9){ //左ボタンでトグル
            mode_flag = !mode_flag; //true = nomal  false = drift
            if(mode_flag == true){
                start_yaw = -Yaw_ahrs;
                target_yaw = 0; 
            }
            delay(500); //チャタリング防止
        }

        if(Stick[THROTTLE] > 0.9){ //THROTTLE上でRoll_bias UP
            Roll_bias = Roll_bias - 0.2;
            delay(100); //連続up　前傾
        }

        if(Stick[THROTTLE] < -0.9){ //THROTTLE下でRoll_bias Down
            Roll_bias = Roll_bias + 0.2;
            delay(100); //連続down　後傾
        }

        if(Roll < -70 or Roll > 70){ //転倒したらOFF
            Start_flag = 0;
        }

        TelemetryData.VoltageR = (float)Voltage_r/100.0;
        TelemetryData.mode = (float)mode_flag;
        //telemetry.hppにデータ項目を追加する

        telemetry();
    }
}

void taskFunction(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 5ms の周期

    // 初期化
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // 次の周期まで待機
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        _St = St;
        St = micros();
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

        // current control
        if(Start_flag==1){
            f1 = 7000; //4200.0;//振子の角度に比例して電流を制御
            f2 = 250.0; //200.0;//振子の角速度に比例して電流を制御
            // f3 = 0.0;//-3.0;//モータの角度に比例して電流を制御 0.1
            // f4 = 0.0;//-5.0;//モータの角速度に比例して電流を制御
            float a = Stick[RUDDER];
            float b = -Stick[ELEVATOR];
            float c = Stick[AILERON];
            if( a > -0.05 and a < 0.05){a = 0.0;} //dead band
            if( b > -0.05 and b < 0.05){b = 0.0;} //dead band
            if( c > -0.05 and c < 0.05){c = 0.0;} //dead band

            if(mode_flag == true){ //drift mode
                // float angle_rad = angle * PI / 180.0;
                float angle_rad = target_yaw * PI / 180.0;
                // 回転計算
                float x1 = c * cos(angle_rad) - b * sin(angle_rad);
                float y1 = c * sin(angle_rad) + b * cos(angle_rad);
                c = x1 / 2.0;
                b = y1;

                target_yaw += a * 1;
                if(target_yaw > 360) target_yaw -= 360;
                if(target_yaw < 0) target_yaw += 360;
                diff = -Yaw_ahrs - start_yaw - target_yaw;
                if(diff > 180) diff -= 360;
                if(diff < -180) diff += 360;
                a = -diff / 10;
            }

            float yaw_ref = -360.0 * a / 2; 
            float yaw_err = yaw_ref - Gyro_z;
            U_yaw = yaw_err * 300.0; //回転
            U_v = -b * 100000.0; //200000.0; //前後移動 ELEVATOR
            U_l = c * 100000.0; //左右移動 AILERON
            float C_yaw = c * 60000.0; //左右移動時の回転モーメント補正
            //State feedback control
            U0 = (-f1 * (Roll - Roll_bias) 
                 - f2 * Gyro_x 
                 - f3 * (float)(Pos_r - Pos_bias_r + Pos_l - Pos_bias_l)/2.0 
                 - f4 * (Speed_r + Speed_l + Speed_r2 + Speed_l2)/4.0 );

            Current_ref_r  = (int32_t)(U0 + U_v + U_yaw * 0.6 + U_l + C_yaw);
            Current_ref_r2 = (int32_t)(U0 + U_v + U_yaw       - U_l + C_yaw);
            Current_ref_l  = (int32_t)(U0 + U_v - U_yaw * 0.6 - U_l - C_yaw);
            Current_ref_l2 = (int32_t)(U0 + U_v - U_yaw       + U_l - C_yaw);

            //limit current
            if (Current_ref_r>120000)Current_ref_r=120000;
            else if (Current_ref_r<-120000)Current_ref_r=-120000;
            if (Current_ref_l>120000)Current_ref_l=120000;
            else if (Current_ref_l<-120000)Current_ref_l=-120000;
            if (Current_ref_r2>120000)Current_ref_r2=120000;
            else if (Current_ref_r2<-120000)Current_ref_r2=-120000;
            if (Current_ref_l2>120000)Current_ref_l2=120000;
            else if (Current_ref_l2<-120000)Current_ref_l2=-120000;

            RollerI2C_RIGHT.setCurrent(Current_ref_r);
            RollerI2C_RIGHT2.setCurrent(-Current_ref_r2);
            RollerI2C_LEFT.setCurrent(-Current_ref_l);
            RollerI2C_LEFT2.setCurrent(Current_ref_l2);
        }
        else{
            RollerI2C_RIGHT.setCurrent(0);
            RollerI2C_RIGHT2.setCurrent(0);
            RollerI2C_LEFT.setCurrent(0);
            RollerI2C_LEFT2.setCurrent(0);
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
    rc_init();
    delay(1000);
    printf("Start\n");
    if(RollerI2C_RIGHT.begin(0x64, 8, 7, 400000)==true){
        printf("RollerI2C_RIGHT begin success\n");
    }
    else{
        printf("RollerI2C_RIGHT begin failed\n");
    }
    if(RollerI2C_LEFT.begin(0x65, 8, 7, 400000)==true){
        printf("RollerI2C_LEFT begin success\n");
    }
    else{
        printf("RollerI2C_LEFT begin failed\n");
    }
    if(RollerI2C_RIGHT2.begin(0x66, 8, 7, 400000)==true){
        printf("RollerI2C_RIGHT2 begin success\n");
    }
    else{
        printf("RollerI2C_RIGHT2 begin failed\n");
    }
    if(RollerI2C_LEFT2.begin(0x67, 8, 7, 400000)==true){
        printf("RollerI2C_LEFT2 begin success\n");
    }
    else{
        printf("RollerI2C_LEFT2 begin failed\n");
    }

    RollerI2C_RIGHT.setMode(3);
    RollerI2C_LEFT.setMode(3);
    RollerI2C_RIGHT.setCurrent(0);
    RollerI2C_LEFT.setCurrent(0);
    RollerI2C_RIGHT.setOutput(1);
    RollerI2C_LEFT.setOutput(1);

    RollerI2C_RIGHT2.setMode(3);
    RollerI2C_LEFT2.setMode(3);
    RollerI2C_RIGHT2.setCurrent(0);
    RollerI2C_LEFT2.setCurrent(0);
    RollerI2C_RIGHT2.setOutput(1);
    RollerI2C_LEFT2.setOutput(1);
    // delay(1000);


    // FreeRTOSタスクの作成
    BaseType_t result = xTaskCreateUniversal(
        taskFunction,
        "5ms Periodic Task",
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
        dummyTask,
        "Dummy Task",
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