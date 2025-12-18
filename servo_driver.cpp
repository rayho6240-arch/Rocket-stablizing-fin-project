// Code by Alejandro Choapa
// 這個檔案負責「真正控制伺服馬達」
// 將飛控算出的脈衝寬度（微秒 us）轉成 PCA9685 可以輸出的 PWM 訊號

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>   // PCA9685 16-channel PWM driver
#include "config.hpp"                  // 所有通道、頻率、servo 參數定義
#include "servo_driver.hpp"

// 建立一個 PCA9685 控制器物件
// I2C 位址由 config.hpp 定義
static Adafruit_PWMServoDriver pwm(PCA9685_I2C_ADDR);

/*
 * 將「微秒 us」轉成 PCA9685 使用的 tick 數（0~4095）
 * PCA9685 一個週期被切成 4096 格
 *
 * 一般 servo：
 *   1000us → 左極限
 *   1500us → 中立
 *   2000us → 右極限
 */
static inline uint16_t usToTicks(int us) {

  // 硬限制，避免寫入不合理數值（保護硬體）
  if (us < 500)  us = 500;
  if (us > 2500) us = 2500;

  // 50Hz 下：4096 ticks / 20ms ≈ 0.2048 tick / us
  float ticks = us * 0.2048f;

  // 再保險一次
  if (ticks < 0)    ticks = 0;
  if (ticks > 4095) ticks = 4095;

  return (uint16_t)(ticks + 0.5f);  // 四捨五入
}

/*
 * 初始化 PCA9685
 * - 啟動 I2C
 * - 設定 PWM 頻率（通常 50Hz 給 servo）
 */
void servosInit() {
  pwm.begin();
  pwm.setPWMFreq(PCA9685_SERVO_HZ);
  delay(10);   // 等待晶片穩定
}

/*
 * 把目前使用到的五個 servo 全部打回「中立位置」
 * 用在：
 * - 開機
 * - fail-safe
 * - 地面測試
 */
void servosNeutral() {

  // 只歸中我們用到的五個通道
  pwm.setPWM(CH_AIL_L, 0, usToTicks(SERVO_MID_US));
  pwm.setPWM(CH_AIL_R, 0, usToTicks(SERVO_MID_US));
  pwm.setPWM(CH_ELE_L, 0, usToTicks(SERVO_MID_US));
  pwm.setPWM(CH_ELE_R, 0, usToTicks(SERVO_MID_US));
  pwm.setPWM(CH_RUD  , 0, usToTicks(SERVO_MID_US));
}

/*
 * 最底層的 servo 輸出函式
 * ch = PCA9685 通道
 * us = 脈衝寬度（微秒）
 */
void servoWriteUs(uint8_t ch, int us) {

  // 先依設定值限制範圍（保護伺服）
  if (us < SERVO_MIN_US) us = SERVO_MIN_US;
  if (us > SERVO_MAX_US) us = SERVO_MAX_US;

  // 寫入 PCA9685
  pwm.setPWM(ch, 0, usToTicks(us));
}

/*
 * 純工具函式
 * 只負責把 us 限制在合法範圍內
 */
int servoClampUs(int us) {
  if (us < SERVO_MIN_US) return SERVO_MIN_US;
  if (us > SERVO_MAX_US) return SERVO_MAX_US;
  return us;
}

/*
 * Servo 形狀處理（非常重要）
 *
 * baseUs : 飛控算出來的指令
 * dir    : 馬達方向（+1 或 -1）
 * trimUs : 機械微調
 *
 * 用途：
 * - 解決左右 servo 方向相反
 * - 修正安裝偏移
 */
int servoShapeUs(int baseUs, int dir, int trimUs) {

  // 算出偏移量（相對於中立）
  int delta = baseUs - SERVO_MID_US;

  // 套用方向與 trim
  int shaped = SERVO_MID_US + (dir * delta) + trimUs;

  // 最後再保護一次
  return servoClampUs(shaped);
}

/*
 * 副翼差動（Differential Aileron）
 *
 * 上偏轉比較大，下偏轉比較小
 * - 用來減少不必要的 yaw
 *
 * 對火箭來說：
 * - 這個函式可以直接刪掉或固定比例
 */
int servoAileronDiffUs(int cmdUs) {

  int delta = cmdUs - SERVO_MID_US;

  if (delta > 0) {
    // 上偏轉 → 加大
    delta = (int)(delta * (1.0f + AIL_DIFF));
  } else {
    // 下偏轉 → 減小
    delta = (int)(delta * (1.0f - AIL_DIFF));
  }

  return servoClampUs(SERVO_MID_US + delta);
}
