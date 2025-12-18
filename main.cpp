// Code by Alejandro Choapa
// main.cpp: 最小化入口程式，實現 IMU + Stabilizer 固件，控制 5 個伺服 + EMI 讀取

#include <Arduino.h>     // Arduino 核心函式庫
#include <Wire.h>        // I2C 通訊函式庫，用於 MPU6050 等感測器

#include "config.hpp"    // 專案設定檔，包含 servo pin、PID 參數等
#include "types.hpp"     // 自訂資料型態，例如 Attitude 結構
#include "imu.hpp"       // IMU 模組，用來讀取姿態角
#include "stabilizer.hpp" // Stabilizer 模組，用 PID 控制伺服保持穩定

// 建立 IMU 物件
static IMU imu;

// 建立穩定器物件（用於控制 5 個伺服）
static Stabilizer stab;

void setup() {
  Serial.begin(115200); // 初始化序列埠，方便 debug 或輸出 telemetry
  Wire.begin();         // 初始化 I2C

  // 初始化 IMU（地址 0x68）
  if (!imu.begin(0x68)) {
    Serial.println(F("ERROR: IMU init failed")); // 若初始化失敗，印錯誤訊息
    while (true) { delay(500); } // 無限迴圈停住，避免程式繼續跑
  }

  // 將 5 個控制面對應到 PCA9685 PWM 通道
  // CH_AIL_L 左副翼, CH_AIL_R 右副翼
  // CH_ELE_L 左升降舵, CH_ELE_R 右升降舵
  // CH_RUD 舵面
  stab.begin(CH_AIL_L, CH_AIL_R, CH_ELE_L, CH_ELE_R, CH_RUD);

  // 設定伺服行程限制與中立脈衝
  // SERVO_MID_US 中立脈衝, SERVO_MIN_US 最小脈衝, SERVO_MAX_US 最大脈衝
  stab.setLimits(SERVO_MID_US, SERVO_MIN_US, SERVO_MAX_US);

  // 設定機械方向，控制輸出正負對應
  // +1 或 -1 用來翻轉控制方向，依照機構安裝方向
  stab.setPolarity(+1, -1, +1, +1, +1);

  // EMI (電磁干擾) 腳位
  pinMode(PIN_EMI, INPUT);
}

void loop() {
  // 計算 delta time（時間間隔）
  static uint32_t last_us = micros();        // 上一次的時間（微秒）
  const uint32_t now_us = micros();          // 現在時間
  const float dt = (now_us - last_us) * 1e-6f; // 將微秒轉成秒
  last_us = now_us;                          // 更新 last_us

  // --- 讀取 IMU，更新姿態 ---
  imu.update(dt);                            // 更新 IMU 資料
  const Attitude& att = imu.attitude();      // 取得最新姿態角 (roll, pitch, yaw)

  // --- 使用 PID 控制 5 個伺服 ---
  stab.step(dt, att);                        // 將 dt 和姿態角傳給穩定器，計算控制量並輸出到伺服

  // --- EMI 讀取 ---
  const int raw = analogRead(PIN_EMI);       // 讀取模擬值 0~1023
  const float emiNorm = raw / 1023.0f;       // 標準化到 0~1

  // 若 EMI 過高，印出警告，可用於 failsafe 或 telemetry
  if (emiNorm > EMI_ALERT_LEVEL) {
    Serial.print(F("EMI HIGH: "));
    Serial.println(emiNorm, 3);
  }

  // --- 輕量級 telemetry，印出 roll/pitch/yaw + EMI ---
  // Serial.print(att.roll); Serial.print(',');
  // Serial.print(att.pitch); Serial.print(',');
  // Serial.print(att.yaw); Serial.print(',');
  // Serial.println(emiNorm, 3);
}
